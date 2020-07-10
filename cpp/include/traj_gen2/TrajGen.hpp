# ifndef TRAJ_GEN
# define TRAJ_GEN

//////////////////////////////////////////////////////////////////////////////////
// TODO : Preventing T imposed pins (t,d same, X different)
// TODO : check this works well with angular elements
//////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <set>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <qpOASES.hpp>
#include <Eigen/LU>

using namespace Eigen;
using namespace std;

/**
 * Trajectory generation modules
 */
namespace trajgen {



    /////////////
    // Typedef //
    /////////////

    typedef unsigned int uint;
    template<typename T,size_t Size> using Vector = Eigen::Matrix<T, Size, 1>;
    template<typename T> using VectorX = Eigen::Matrix<T, -1, 1>;
    typedef uint d_order; // derivative order
    typedef uint p_order; // polynomial order
    template<typename T> using time_knots = vector<T> ;
    template<typename T> using spMatrixRow = SparseMatrix<T,RowMajor>;
    template<typename T> using PolyCoeff =  Eigen::Matrix<T, -1, 1>;
    template<typename T> using MatrixRow = Matrix<T,-1,-1,RowMajor>;

    ///////////////
    // Sub utils //
    ///////////////

    /**
     * @brief Insert inMat to targetMat.block(startRow,startCol,rowSize,colSize).
     * @details As the block insertion for sparse matrix is not implemented in Eigen3, I wrote.
     * @param targetMat Insertion target (big)
     * @param inMat Inserted matrix (small)
     * @param startRow
     * @param startCol
     */
    template <typename T>
    void sparseBlockCopy(SparseMatrix<T,RowMajor> *targetMat, SparseMatrix<T,RowMajor> inMat, long startRow, long startCol) {
        for (int k = 0; k < inMat.rows(); ++k)
            for (typename SparseMatrix<T,RowMajor>::InnerIterator it(inMat, k); it; ++it) { // we use sparsity
                long insertR = it.row() + startRow;
                long insertC = it.col() + startCol;
                targetMat->coeffRef(insertR, insertC) = it.value();
            }
    }
    /**
     * @brief Insert inMat to targetMat.block(startRow,startCol,rowSize,colSize). As the block insertion for sparse matrix is not implemented in Eigen3, I wrote.
     * @param targetMat Insertion target (big)
     * @param inMat Inserted matrix (small)
     * @param startRow
     * @param startCol
     */
    template <typename T>
    void sparseBlockCopy(SparseMatrix<T,RowMajor> *targetMat, SparseMatrix<T,ColMajor> inMat, long startRow, long startCol) {
        for (int k = 0; k < inMat.cols(); ++k)
            for (typename SparseMatrix<T,ColMajor>::InnerIterator it(inMat, k); it; ++it) { // we use sparsity
                long insertR = it.row() + startRow;
                long insertC = it.col() + startCol;
                targetMat->coeffRef(insertR, insertC) = it.value();
            }
    }
    /**
     * @brief Insert inMat to targetMat.block(startRow,startCol,rowSize,colSize). As the block insertion for sparse matrix is not implemented in Eigen3, I wrote.
     * @param targetMat Insertion target (big)
     * @param inMat Inserted matrix (small)
     * @param startRow
     * @param startCol
     */
    template <typename T>
    void sparseBlockCopy(SparseMatrix<T,RowMajor> *targetMat, Matrix<T,-1,-1,RowMajor> inMat, long startRow, long startCol) {
        for (int k = 0; k < inMat.rows(); ++k)
            for (typename Matrix<T,-1,-1,RowMajor>::InnerIterator it(inMat, k); it; ++it) { // we use sparsity
                long insertR = it.row() + startRow;
                long insertC = it.col() + startCol;
                targetMat->coeffRef(insertR, insertC) = it.value();
            }
    }

    /**
     * @brief extract the diff element (setA - setB).
     * @tparam T
     * @param setA
     * @param setB
     * @return
     */
    template <typename T> vector<T> setDiff(const set<T> & setA, const set<T> & setB){
        vector<T> output_it(setA.size());
        auto it =std::set_difference(setA.begin(),setA.end(),
                setB.begin(),setB.end(),output_it.begin());

        output_it.resize(it-output_it.begin());
        return output_it;
    }
    /**
     * @brief Perform interpolation for the data pair (xData,yData) given a query value x
     * @tparam T
     * @param xData
     * @param yData
     * @param x
     * @param extrapolate if true, extrapolation
     * @return
     */
    template <typename T>
    T interpolate(const VectorX<T>& xData, const VectorX<T>& yData,const T & x,bool extrapolate){
        int size = xData.size();
        int i = 0;                                                                  // find left end of interval for interpolation
        if ( x >= xData(size - 2) )                                                 // special case: beyond right end
        {
            i = size - 2;
        }
        else
        {
            while ( x > xData(i+1) ) i++;
        }
        T xL = xData(i), yL = yData(i), xR = xData(i+1), yR = yData(i+1);      // points on either side (unless beyond ends)
        if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
        {
            if ( x < xL ) yR = yL;
            if ( x > xR ) yL = yR;
        }

        T dydx = ( yR - yL ) / ( xR - xL );                                    // gradient
        return yL + dydx * ( x - xL );                                              // linear interpolation
    }


    /////////////////////////////////////////////////
    // PIN for equality or inequality constriants  //
    /////////////////////////////////////////////////


    enum PIN_TYPE {
        FIX_PIN = 0, // equality constraint
        LOOSE_PIN = 1 // inequality constraint
    };
    /**
     * @brief Pin object represent the constraints to be imposed on trajectory.
     * @tparam T datatype (e.g. float or double). The choice of the datatype critical for the accuaracy
     * @tparam dim dimension of operational space (e.g 2D or 3D)
     */
    template<typename T,size_t dim>
    struct Pin{
        T t; // imposed time
        uint d; // imposed order of derivative
        Pin(T t_, d_order d_) : t(t_), d(d_) {};
        virtual PIN_TYPE getType() const = 0;
    };
    /**
     * @brief Equality constraint
     * @tparam T
     * @tparam dim
     */
    template<typename T,size_t dim>
    struct FixPin : public Pin<T,dim> {
        Vector<T,dim> x; // waypoint
        FixPin(T t_, uint d_, Vector<T,dim> x_) : Pin<T,dim>(t_, d_), x(x_) {};
        PIN_TYPE getType() const { return PIN_TYPE::FIX_PIN; }
    };

    /**
     * @brief Inequality constraint
     * @tparam T
     * @tparam dim
     */
    template<typename T,size_t dim>
    struct LoosePin : public Pin<T,dim> {
        Vector<T,dim> xl; // lower bound
        Vector<T,dim> xu; // upper bound
        LoosePin(T t_, uint d_, Vector<T,dim> xl_, Vector<T,dim> xu_) : Pin<T,dim>(t_, d_), xl(xl_), xu(xu_) {};
        PIN_TYPE getType() const { return PIN_TYPE::LOOSE_PIN; }
    };

    /////////////////////////////////////
    // Quadratic programming structure //
    /////////////////////////////////////
    /**
     * @brief Constraint matrix pair (Ax = b or Ax <= b). A is assumed to be sparse while b to be as dense matrix.
     * @tparam T
     * @tparam dim
     */
    template<typename T,size_t dim>
    struct ConstraintMatPair {
        vector<spMatrixRow<T>> ASet;
        vector<MatrixRow<T>> bSet;

        ConstraintMatPair() {
            ASet.resize(dim), bSet.resize(dim);
        }
        /**
         * @brief Intialize the size of matrix AB
         * @param Nc number of constraints (row)
         * @param nVar number of variable of the optimization (column)
         */
        void initialize(uint Nc, uint nVar) {
            for (spMatrixRow<T> &sp : ASet)
                sp = spMatrixRow<T>(Nc, nVar);
            for (MatrixRow<T> &mat : bSet)
                mat = MatrixRow<T>::Zero(Nc, 1);
        }
        ConstraintMatPair(uint Nc, uint nVar) {
            ASet.resize(dim), bSet.resize(dim);
            this->initialize(Nc, nVar);
        }
    };

    /**
     * @brief The basic qp block (p'Qp + Hp). Note that 1/2 is not multiplied!
     * @tparam T
     */
    template<typename T>
    struct QpBlock {
        spMatrixRow<T> Q;
        spMatrixRow<T> H;
        spMatrixRow<T> A;
        spMatrixRow<T> b;
        spMatrixRow<T> Aeq;
        spMatrixRow<T> beq;
        QpBlock(uint nVar, uint neq, uint nineq);
        QpBlock() {};
        VectorX<T> getQpSol(bool & isSolved);
    };

    /**
     * @brief Dimensional collection of qp Blocks.
     * @tparam T
     * @tparam dim
     */
    template<typename T,size_t dim>
    struct QpForm {
        uint nVar,neq,nineq;
        QpBlock<T> *qpBlock;
        QpForm(uint nVar, uint neq, uint nineq) : nVar(nVar),neq(neq),nineq(nineq){
            qpBlock = new QpBlock<T>[dim];
            for (uint d = 0; d < dim; d++)
                qpBlock[d] = QpBlock<T>(nVar, neq, nineq);
        };
        void write();
        vector<VectorX<T>> getQpSolSet(bool & isSolved);
    };

    /////////////////////////////
    // TrajGen as a base class //
    /////////////////////////////
    /**
     * @brief TrajGen is abstract class as a base for polyTrajGen and optimTrajGen
     * @tparam T datatype (e.g. float or double). The choice of the datatype critical for the accuaracy
     * @tparam dim dimension of operational space (e.g. 2D or 3D)
     */
    template<typename T,size_t dim>
    class TrajGen {

    protected:
        uint M; // number of segment
        bool isSolved = false; // solve flag
        time_knots<T> ts; // time knots (do not be confused with time points in case of optimTrajGen)

        vector<FixPin<T,dim>> *fixPinSet;  // equality constraints set of M segment. (In case of optimTrajGen, M = 1)
        vector<LoosePin<T,dim>> *loosePinSet; // inequality constraints set of ()
        set<d_order> *fixPinOrderSet;
        VectorX<T> weight_mask; // high order derivative penalty weights

        // Subroutine functions
        void findSegInterval(T t, uint &m);
        void findSegInterval(T t, uint &m, T &tau);

        virtual ConstraintMatPair<T,dim> fixPinMatSet(const FixPin<T,dim> *pPin,uint blockSize = 1) = 0;
        virtual ConstraintMatPair<T,dim> loosePinMatSet(const LoosePin<T,dim> *pPin,uint blockSize = 1) = 0;
        virtual QpForm<T,dim> getQPSet() = 0; // create qp problem with current information
        uint getTotalNineq();
        virtual uint getTotalNeq(); // different for polyTrajGen and optimTrajGen (in case of optimTrajGen = TrajGen)

    public:
        TrajGen(time_knots<T> ts_);
        TrajGen() {};
        void addPinSet(const vector<Pin<T,dim> *> &pinPtrSet);
        void setDerivativeObj(VectorX<T> weights);
        virtual void addPin(const Pin<T,dim> *pin);
        virtual Vector<T,dim> eval(T t_eval, d_order d) = 0;
        virtual bool solve(bool verbose) = 0;
    };

    ///////////////////////////////////////////
    // PolyTrajGen for piecewise polynomials //
    ///////////////////////////////////////////
    /**
     * @brief Solving method for polynomial
     * @details POLY_COEFF = optimization w.r.t poly coefficient. Fast but unstable for large problem. END_DERIVATIVE = vice versa
     */
    enum ALGORITHM{
        POLY_COEFF = 0,
        END_DERIVATIVE = 1
    };

    /**
     * @brief PolyParameters to define the behavior of polynomial
     */
    struct PolyParam{
        p_order poly_order = 4;
        d_order max_conti_order = 2;
        ALGORITHM algo = ALGORITHM::POLY_COEFF;
        PolyParam() { printf("No default parameters given. Default values are used\n");};
        PolyParam(p_order N,d_order max_conti,ALGORITHM algo): poly_order(N),max_conti_order(max_conti),algo(algo) {};
    };

    /**
     * @brief State of the polynomial segment (number of imposed continuity, added fixed pin).
     */
    struct PolyState{
        uint Nc = 0; // order of continuity
        uint Nf = 0; // number of fixed pins
        uint getN() {return Nc+Nf;};
    };
    /**
     * @brief Piecewise polynomial trajectory. In this method, we define the primitive of trajectory as polynomial.
     * @details Optimization is performed w.r.t the coefficient of polynomials or free end-derivative of each polynomial segment
     * @tparam T
     * @tparam dim
     */
    template<typename T,size_t dim>
    class PolyTrajGen : public TrajGen<T,dim>{

        private:
            PolyParam param;
            PolyState* seg_state_set;
            PolyCoeff<T>** poly_coeff_set; // [dim][M]
            p_order N; // poly order

            // subroutine functions
            T B(p_order n,d_order d);
            VectorX<T> tVec(T t,d_order d);
            spMatrixRow<T> scaleMat(T dt);
            spMatrixRow<T> scaleMatBig();
            spMatrixRow<T> scaleMatBigInv();

            // objective
            spMatrixRow<T> IntDerSquard(d_order d);
            // constraints
            ConstraintMatPair<T,dim> contiMat(uint m, d_order dmax);
            ConstraintMatPair<T,dim> fixPinMatSet(const FixPin<T,dim> * pPin,uint blockSize = 1);
            ConstraintMatPair<T,dim> loosePinMatSet(const LoosePin<T,dim> * pPin,uint blockSize = 1);
            uint getTotalNeq(); // overloaded
            // qp
            QpForm<T,dim> getQPSet();
            // map poly-ceoff to end-derivatives
            MatrixRow<T> coeff2endDerivatives(const spMatrixRow<T> & Aeq);
            QpForm<T,dim> mapQP(const QpForm<T,dim>& QpFormOrig);

    public:
            PolyTrajGen(time_knots<T> ts_,PolyParam param_) ;
            void addPin(const Pin<T,dim>* pin);
            Vector<T,dim> eval(T t_eval, d_order d);
            bool solve(bool verbose);
    };

    ///////////////////////////////
    // Optimal trajectory method //
    ///////////////////////////////
    /**
     * @brief The trajectory is represented as the linear interpolation of discrete points
     * @details In this method, the optimization variables are coordinate of each point
     * @tparam T
     * @tparam dim
     */
    template<typename T,size_t dim>
    class OptimTrajGen : public TrajGen<T,dim>{
        private:
            // member variables
            float pntDensity;
            uint nPnt;
            T dt;
            time_knots<T> tPnts; // time points
            VectorX<T>* trajPnts; // trajectory point. pointer = dim / VectorX<T> is element-wise trajectory

            // subroutine
            ConstraintMatPair<T,dim> fixPinMatSet(const FixPin<T,dim> * pPin,uint blockSize = 1);
            ConstraintMatPair<T,dim> loosePinMatSet(const LoosePin<T,dim> * pPin,uint blockSize = 1);
            uint matchIdx(T eval_t,d_order d = 0); // get the corresponding time point index of a evaluation time
            spMatrixRow<T> getDiff(d_order d);
            QpForm<T,dim> getQPSet();

        public:
            OptimTrajGen(time_knots<T> ts_, float pntDensity);
            void addPin(const Pin<T,dim>* pin);
            bool solve(bool isPrint);
            Vector<T,dim> eval(T t_eval, d_order d);
    };










    ///////////////////////////////////////////////////////////////////////////////////////

           // FUNCTION DEFINITIONS (as we use template, better to include defs here) //

    ///////////////////////////////////////////////////////////////////////////////////////



    /////////////////////////////////////
    // Quadratic programming structure //
    /////////////////////////////////////

    /**
     * write current problem and save it as txt file
     */
    template<typename T,size_t dim> void QpForm<T,dim>::write() {
        ofstream file ("QP");
        file << "Q:" << "\n" << MatrixRow<T>(qpBlock[0].Q) << endl;
        for (uint dd = 0 ; dd < dim ; dd++)
            file << "H[" << dd << "]:" << "\n" << MatrixRow<T>(qpBlock[dd].H) << endl;
        file << "A:" << "\n" << MatrixRow<T>(qpBlock[0].A) << endl;
        for (uint dd = 0 ; dd < dim ; dd++)
            file << "b[" << dd << "]:\n" << MatrixRow<T>(qpBlock[dd].b) << endl;

        file << "Aeq:" << "\n" << MatrixRow<T>(qpBlock[0].Aeq) << endl;
        for (uint dd = 0 ; dd < dim ; dd++)
            file << "beq[" << dd << "]:\n" << MatrixRow<T>(qpBlock[dd].beq) << endl;
        file.close();
    }

    template <typename T,size_t dim> vector<VectorX<T>> QpForm<T,dim>::getQpSolSet(bool & isSolved) {
        vector<VectorX<T>> qpSolSet(dim);
        isSolved = true;
        for (uint dd = 0; dd < dim ; dd++) {
            qpSolSet[dd] = qpBlock[dd].getQpSol(isSolved);
            if (not isSolved)
                break;
        }
        return qpSolSet;
    }


    template<typename T>
    QpBlock<T>::QpBlock(uint nVar, uint neq, uint nineq) {
        Q = spMatrixRow<T>(nVar, nVar);
        H = spMatrixRow<T>(1, nVar);
        A = spMatrixRow<T>(nineq, nVar);
        b = spMatrixRow<T>(nineq, 1);
        Aeq = spMatrixRow<T>(neq, nVar);
        beq = spMatrixRow<T>(neq, 1);
    }
    /**
     * @brief solve the current quadratic programming with qpOASES.
     * @tparam T
     * @param isSolved ref returned with result
     * @return
     */
    template<typename T>
    VectorX<T> QpBlock<T>::getQpSol(bool & isSolved) {
        uint nVar = Q.rows();
        int N_ineq_const = A.rows();
        int N_eq_const = Aeq.rows();
        int N_const = N_ineq_const + N_eq_const;
        // convention follows the qpOASES
        qpOASES::QProblem qp_obj(nVar, N_const, qpOASES::HST_SEMIDEF);
        Matrix<double, -1, -1, RowMajor> Hmat(2 * Q.template cast<double>());
        Matrix<double, 1, -1, RowMajor> gmat(H.template cast<double>());
        Matrix<double, -1, -1, RowMajor> Amat(N_const, nVar);
        Amat << MatrixXd(A.template cast<double>()), MatrixXd(Aeq.template cast<double>());
        Matrix<double, -1, -1, RowMajor> ubAmat(N_const, 1);
        ubAmat << MatrixXd(b.template cast<double>()), MatrixXd(beq.template cast<double>());
        MatrixXd MinusInf(N_ineq_const, 1);
        MinusInf=MinusInf.setConstant(-99999);
        Matrix<double, -1, -1, RowMajor> lbAmat(N_const, 1);
        lbAmat << MinusInf, MatrixXd(beq.template cast<double>());

        qpOASES::real_t *Hqp = Hmat.data();
        qpOASES::real_t *gqp = gmat.data();
        qpOASES::real_t *Aqp = Amat.data();
        qpOASES::real_t *lbAqp = lbAmat.data();
        qpOASES::real_t *ubAqp = ubAmat.data();

        qpOASES::Options options;
        options.printLevel = qpOASES::PL_LOW;
        options.terminationTolerance = 1e-10;
//        options.enableRamping=qpOASES::BT_FALSE;
        qp_obj.setOptions(options);
        qpOASES::int_t nWSR = 4000;
        qp_obj.init(Hqp, gqp, Aqp, NULL, NULL, lbAqp, ubAqp, nWSR);
        if(qp_obj.isInfeasible()){
            cout<<"[QP solver] warning: problem is infeasible. "<<endl;
        }

        cout << "Acutual nWSR: " << nWSR << " / Original: " << 2000 << endl;

        // qpOASES::real_t xOpt[nVar];
        // problem by VS, C2131
        qpOASES::real_t* xOpt = new qpOASES::real_t[nVar];
        qp_obj.getPrimalSolution(xOpt);
        isSolved = qp_obj.isSolved();
        if (isSolved)
            cout<<"[QP solver] Success!  "<<endl;
        else

            cout<<"[QP solver] Failure.  "<<endl;
        return Map<Matrix<double, -1, 1>>(xOpt, nVar, 1).cast<T>();
    }


    /////////////////////////////
    // TrajGen as a base class //
    /////////////////////////////
    template <typename T,size_t dim> TrajGen<T,dim>::TrajGen(time_knots<T> ts_): ts(ts_){
        // pinset per segment
        M = ts.size()-1; // # of segment = length(knots) - 1
        this->fixPinSet = new vector<FixPin<T,dim>>[M];
        this->loosePinSet = new vector<LoosePin<T,dim>>[M];
        fixPinOrderSet = new set<d_order>[M];
        printf ("Initialized %d segments. \n",ts_.size()-1);
    }

    /**
     * @brief Find a segment index (0,1,...,M-1) which contains t (ts[m]<= t < ts[m+1]  || t< ts[1] || ts[M-1]<= t)
     * @tparam dim
     * @param t query time
     * @param m segment index
     */
    template<typename T,size_t dim> void TrajGen<T,dim>::findSegInterval(T t,uint& m){
        auto it = upper_bound(this->ts.begin(),this->ts.end(),t);
        m = std::min(std::max( ((it - this->ts.begin())-1),long(0)),long(this->M-1)); // extrapolation allowed.
    }

    /**
     * Find a segment index (0,1,...,M-1) which contains t (ts[m]<= t < ts[m+1]  || t< ts[1] || ts[M-1]<= t)
     * @tparam dim
     * @param t query time
     * @param m segment index
     * @param tau Normalized time on the segment
     */
    template <typename T,size_t dim> void TrajGen<T,dim>::findSegInterval(T t, uint &m, T &tau) {
        findSegInterval(t,m);
        tau = (t - this->ts[m]) / (this->ts[m+1] - this->ts[m]);
        if (tau<0 || tau > 1)
            printf("tau is outside of [0,1]\n");
    }


    /**
     * Add pin struct to corresponding segment
     * @tparam T
     * @tparam dim
     * @param pin
     */
    template<typename T,size_t dim> void TrajGen<T,dim>::addPin(const Pin<T,dim> *pin){
        uint m;
        this->findSegInterval(pin->t,m); // which segment to add the pin
        if (pin->getType() == PIN_TYPE::FIX_PIN)
            fixPinSet[m].push_back(*(static_cast<const FixPin<T,dim>*>(pin)));
        else
            loosePinSet[m].push_back(*(static_cast<const LoosePin<T,dim>*>(pin)));
    }
    /**
     * This method process the set of pins in one-shot way
     * @tparam T
     * @tparam dim
     * @param pinPtrSet
     */
    template<typename T,size_t dim> void TrajGen<T,dim>::addPinSet(const vector<Pin<T,dim>*>& pinPtrSet){
        for (auto it = pinPtrSet.begin() ; it < pinPtrSet.end() ; it++)
            addPin(*it);
    }
    /**
     * Set the weights of each component of derivative penalty
     * @tparam T
     * @tparam dim
     * @param weights
     */
    template<typename T,size_t dim> void TrajGen<T,dim>::setDerivativeObj(VectorX<T> weights){
        weight_mask = weights;
    }
    /**
     *     get total number of inequality condition.
     */
    template<typename T,size_t dim> uint TrajGen<T,dim>::getTotalNineq() {
        uint nineq = 0;
        for (uint m = 0 ; m < M ; m++)
            nineq+= 2*loosePinSet[m].size();
        return nineq;
    }
    /**
     * get total number of equality condition.
     * @tparam T
     * @tparam dim
     * @return
     */
    template<typename T,size_t dim> uint TrajGen<T,dim>::getTotalNeq() {
        uint neq = 0;
        for (uint m = 0 ; m < M ; m++)
            neq+= fixPinSet[m].size();
        return neq;
    }


    ///////////////////////////////////////////
    // PolyTrajGen for piecewise polynomials //
    ///////////////////////////////////////////

    template<typename T,size_t dim> PolyTrajGen<T,dim>::PolyTrajGen(time_knots<T> ts_,PolyParam param_)  : TrajGen<T,dim>(ts_), param(param_),N(param_.poly_order){

        seg_state_set = new PolyState[this->M];

        // polycoeff segment / dimension
        poly_coeff_set = new PolyCoeff<T>*[dim];
        for (int dd = 0;dd <dim ; dd++)
            poly_coeff_set[dd] = new PolyCoeff<T>[this->M];

        // set the default Nc (except the last segment )
        for (uint m = 0 ; m < (this->M - 1); m++)
            seg_state_set[m].Nc = param_.max_conti_order+1;
    }


    template<typename T,size_t dim> void PolyTrajGen<T,dim>::addPin(const Pin<T,dim>* pin){
        // ensure the pin is imposed on the knots of poly spline
        uint m;
        this->findSegInterval(pin->t,m);
        if (pin->getType() == PIN_TYPE ::FIX_PIN)
            assert(this->ts[m] == pin->t || this->ts[m+1] == pin->t && "For polyTrajGen, fix pins can be imposed only on the knots.");

        // add pin
        if (pin->getType() == PIN_TYPE::FIX_PIN)
            if (seg_state_set[m].Nf > N + 1)
                printf("Fix pin already full for %d th segment. Pin append is ignored. (Nf = %d / max DOF = %d)\n"
                        ,m,seg_state_set[m].Nf,N+1);
            else{
                // add pin
                TrajGen<T,dim>::addPin(pin);
                seg_state_set[m].Nf+= 1;
                if (seg_state_set[m].Nf +seg_state_set[m].Nc > N+1 ) // if this pin addition overflow the total # of dof
                {
                    seg_state_set[m].Nc -= 1;
                    printf ("Fix pin reduced the continuity order (seg : %d, Nf=%d / Nc=%d).\n",seg_state_set[m].Nf,seg_state_set[m].Nc);
                }
                this->fixPinOrderSet[m].insert(pin->d);
            }
        else
            TrajGen<T,dim>::addPin(pin);
    }

    /**
     * The nth order coefficient of time vector ([1 t t^2 ... t^N]) when differentiated d times.
     * @tparam dim
     * @param n : target order
     * @param d : order of derivative
     * @return
     */
    template <typename T,size_t dim> T PolyTrajGen<T,dim>::B(p_order n,d_order d){
        if (d==0)
            return 1;
        else
            if (n<d)
                return 0;
            else{
                VectorXi prodTarget = VectorXi::LinSpaced(d,n-(d-1),n);
                return accumulate(prodTarget.data(),prodTarget.data()+prodTarget.size(),
                        1,std::multiplies<int>());
            }
    }
    /**
     * time vector ([1 t t^2 ... t^N]) when differentiated d times.
     * @tparam dim
     * @param t
     * @param d
     * @return
     */
    template <typename T,size_t dim> VectorX<T> PolyTrajGen<T,dim>::tVec(T t,d_order d){
        VectorX<T> vec(this->N+1); vec.setZero();
        for (int n = d ; n < N+1 ; n ++)
            vec(n) = B(n,d)*pow(t,n-d);
        return vec;
    }
    /**
     * p = scaleMat(dt) * phat
     * @tparam dim
     * @param dt segment duration (dTm)
     * @return
     */
    template <typename T,size_t dim> spMatrixRow<T> PolyTrajGen<T,dim>::scaleMat(T dt){
        spMatrixRow<T> mat(N+1,N+1);

        vector<Triplet<T>> tripletList;
        for (int i = 0 ; i < N+1 ; i++)
            tripletList.emplace_back(i,i,pow(dt,i));
        mat.setFromTriplets(tripletList.begin(),tripletList.end());
        return mat;
    }
    /**
     * diag([scaleMat(dTm)]_{m=[0,1,..,M-1]}). P = scaleMatBig * Phat
     * @tparam T
     * @tparam dim
     * @return
     */
    template <typename T,size_t dim> spMatrixRow<T> PolyTrajGen<T,dim>::scaleMatBig() {
        spMatrixRow<T> mat(this->M*(N+1),this->M*(N+1));
        vector<Triplet<T>> tripletList;
        for (int m = 0 ; m < this->M ; m++)
            for (int i = 0 ; i < N+1 ; i++)
                tripletList.emplace_back( (N+1)*m + i  ,(N+1)*m + i, pow(this->ts[m+1] - this->ts[m],i));
        mat.setFromTriplets(tripletList.begin(),tripletList.end());
        return mat;
    }
    /**
     * Inverse of scaleMatBig.
     * @tparam T
     * @tparam dim
     * @return
     */
    template <typename T,size_t dim> spMatrixRow<T> PolyTrajGen<T,dim>::scaleMatBigInv() {
        spMatrixRow<T> mat(this->M*(N+1),this->M*(N+1));
        vector<Triplet<T>> tripletList;
        for (int m = 0 ; m < this->M ; m++)
            for (int i = 0 ; i < N+1 ; i++)
                tripletList.emplace_back( (N+1)*m + i  ,(N+1)*m + i, pow(this->ts[m+1] - this->ts[m],-i));
        mat.setFromTriplets(tripletList.begin(),tripletList.end());
        return mat;
    }

    /**
     * {x^(d)(t)}^2  = (tVec(t,d)'*Dp)'*(tVec(t,d)'*Dp)
     * @tparam dim
     * @param d
     * @return
     */
    template <typename T,size_t dim> spMatrixRow<T> PolyTrajGen<T,dim>::IntDerSquard(trajgen::d_order d) {
        spMatrixRow<T> Q(N+1,N+1);
        if (d > N) {
            printf("Order of derivative exceeds the polynomial order. Returning zero mat.\n");
        }else{
            for (uint i = d;i<N+1 ; i++)
                for(uint j = d ; j < N+1 ; j ++)
                    Q.coeffRef(i,j) = B(i,d) * B(j,d) / (i+j-2*d+1);
        }
        return Q;
    }
    /**
     * Convert the stacked pin information to the constraints Aeq*p = beq. We can consider multiple pin at once
     * @tparam dim
     * @param pPin pointer to the first element of the array pin
     * @param blockSize how many pins to be considered
     * @return
     */
    template <typename T,size_t dim> ConstraintMatPair<T,dim> PolyTrajGen<T,dim>::fixPinMatSet(const FixPin<T,dim> *pPin,uint blockSize){
        uint nVar = this->M * (N+1);
        uint Nc = blockSize; // only one
        ConstraintMatPair<T,dim> abEq (Nc,nVar);

        for (uint blk = 0; blk < blockSize ; blk++) {
            uint m; T tau;
            this->findSegInterval((pPin+blk)->t, m, tau);
            uint idxStart = m * (N + 1);
            T dt = this->ts[m + 1] - this->ts[m];
            d_order d = (pPin+blk)->d;
            for (uint dd = 0; dd < dim; dd++) {
                sparseBlockCopy<T>(&(abEq.ASet[dd]),tVec(tau,d).transpose()/pow(dt,d),blk,idxStart);
                abEq.bSet[dd](blk) = (pPin + blk)->x(dd);
            }
        }
        return abEq;
    }

    /**
     * Convert the stacked pin information to the constraints A*p < b. We can consider multiple pin at once
     * @tparam T
     * @tparam dim
     * @param pPin
     * @param blockSize how many pins to be considered? total constraints = 2*pin
     * @return
     */
    template <typename T,size_t dim> ConstraintMatPair<T,dim> PolyTrajGen<T,dim>::loosePinMatSet(const LoosePin<T,dim> *pPin,uint blockSize){
        uint nVar = this->M * (N+1);
        uint Nc = 2*blockSize;
        ConstraintMatPair<T,dim> abInEq (Nc,nVar);
       for (uint blk = 0 ; blk < blockSize ; blk ++) {
           uint m; T tau;
           this->findSegInterval((pPin+blk)->t, m, tau);
           uint idxStart = m * (N + 1);
           T dt = this->ts[m + 1] - this->ts[m];
           d_order d = (pPin+blk)->d;
           for (uint dd = 0; dd < dim; dd++) {

               sparseBlockCopy<T>(&(abInEq.ASet[dd]),tVec(tau,d).transpose()/pow(dt,d),2*blk,idxStart);
               sparseBlockCopy<T>(&(abInEq.ASet[dd]),-tVec(tau,d).transpose()/pow(dt,d),2*blk+1,idxStart);

               abInEq.bSet[dd](2*blk) = (pPin+blk)->xu(dd);
               abInEq.bSet[dd](2*blk+1) = -(pPin+blk)->xl(dd);
           }
       }
        return abInEq;
    }

    /**
     * Equality constraint to connect m segment to m+1 segment (m = 0... M-1)
     * @tparam dim
     * @param m
     * @param dmax : we impose  dmax continuity (0,1,...,dmax). This is given from outside of this method.
     * @return
     */
    template <typename T,size_t dim> ConstraintMatPair<T,dim> PolyTrajGen<T,dim>::contiMat(uint m, trajgen::d_order dmax) {

        uint nVar = this->M *(N+1);

        if (m == this->M-1)
            return ConstraintMatPair<T,dim>(0,nVar);

        uint Nc = dmax + 1;
        ConstraintMatPair<T,dim> abEq(Nc,nVar);
        uint idxStart = m*(N+1);
        T dt1 = this->ts[m+1] - this->ts[m];
        T dt2 = this->ts[m+2] - this->ts[m+1];

        for (uint dd = 0; dd < dim ; dd++)
            for (d_order d = 0; d <= dmax; d++) {
                sparseBlockCopy<T>(&(abEq.ASet[dd]),tVec(1, d).transpose() / pow(dt1, d),d,idxStart);
                sparseBlockCopy<T>(&(abEq.ASet[dd]),-tVec(0, d).transpose() / pow(dt2, d),d,idxStart + (N+1));

                abEq.bSet[dd](d) = 0;
            }

        return abEq;
    }

    /**
     * Get the total number of equality constraint (sum (Nc+Nf) for m = 0,,,M-1)
     * @tparam dim
     * @return
     */
    template<typename T,size_t dim> uint PolyTrajGen<T,dim>::getTotalNeq() {
        auto lambda = [&] (uint s1,PolyState s2){return s1+s2.getN();};
        return accumulate(seg_state_set,seg_state_set+this->M,0,lambda);
    };


    /**
     * construct qp problem
     * @tparam dim
     * @return
     */
    template<typename T,size_t dim> QpForm<T,dim> PolyTrajGen<T,dim>::getQPSet() {

        uint nVar = (this->M)*(N+1);
        uint nineq = this->getTotalNineq(); // total line of inequality
        uint neq = this->getTotalNeq();
        QpForm<T,dim> qpForm(nVar,neq,nineq); // initialize all the container

        // 1. Objective function
        spMatrixRow<T> Q(nVar,nVar);
        for (uint d = 1; d <= this->weight_mask.size(); d++) { // dth order derivative
            spMatrixRow<T> Qd(nVar, nVar);
            for (uint m = 0; m < this->M; m++) { // per segment, plug in the Q
                T dt = this->ts[m + 1] - this->ts[m];
                sparseBlockCopy<T>(&Qd,MatrixRow<T>(IntDerSquard(d)/pow(dt,2*d-1)),m*(N+1),m*(N+1));
            }
            Q += this->weight_mask[d - 1] * Qd;
        }

        for (uint dd = 0 ; dd < dim ; dd++)
            qpForm.qpBlock[dd].Q = Q;

        uint Idx1 = 0,Idx2 = 0;
        for (uint m = 0 ; m < this->M ; m++){ // per segment
            // 2. Ineq constraint for this segment
            uint Nineq = this->loosePinSet[m].size();
            ConstraintMatPair<T,dim> Ab = loosePinMatSet((this->loosePinSet[m]).data(),this->loosePinSet[m].size());
            for (uint dd = 0 ; dd < dim ; dd++) { // per dimension
                sparseBlockCopy<T>(&(qpForm.qpBlock[dd].A),(Ab.ASet[dd]),Idx1,0);
                sparseBlockCopy<T>(&(qpForm.qpBlock[dd].b),(Ab.bSet[dd]),Idx1,0);
            }
            Idx1+= 2*Nineq;

            // 3. Eq constraint for this segment
            ConstraintMatPair<T,dim> AbFix = fixPinMatSet((this->fixPinSet[m]).data(),this->fixPinSet[m].size());
            ConstraintMatPair<T,dim> AbConti = contiMat(m,seg_state_set[m].Nc-1);
            uint Neq = seg_state_set[m].getN(), Nf = seg_state_set[m].Nf, Nc = seg_state_set[m].Nc;
            for (uint dd = 0 ; dd < dim ; dd++) { // inserting A,b
                // (Aeq)m = ([Afix ; Aconti])m : Aeq*p = beq
                sparseBlockCopy<T>(&(qpForm.qpBlock[dd].Aeq),(AbFix.ASet[dd]),Idx2,0) ;
                sparseBlockCopy<T>(&(qpForm.qpBlock[dd].Aeq),(AbConti.ASet[dd]),Idx2+Nf,0);
                sparseBlockCopy<T>(&(qpForm.qpBlock[dd].beq),(AbFix.bSet[dd]),Idx2,0) ;
                sparseBlockCopy<T>(&(qpForm.qpBlock[dd].beq),(AbConti.bSet[dd]),Idx2+Nf,0);

            }
            Idx2 += Neq;
        }
        return qpForm;
    }

    /**
     * Solving routine using qpOASES
     * @tparam dim
     * @return
     */
    template <typename T,size_t dim > bool PolyTrajGen<T,dim>::solve(bool isPrint){

        if (TrajGen<T,dim>::getTotalNeq() == 0 and this->getTotalNineq() == 0) {
            cout << "No constraints exist. Aborting. " << endl;
            return false;
        }

        // Prepare QP
        QpForm<T,dim> qpForm = getQPSet();
        QpForm<T,dim> qpFormOrig = qpForm;
        // If this method is end-derivative, we should map to end-derivative
        if (param.algo == ALGORITHM::END_DERIVATIVE)
            qpForm = mapQP(qpForm);

        qpForm.write();

        // Solve
        bool isSolved; VectorX<T> P;
        vector<VectorX<T>> solSet = qpForm.getQpSolSet(isSolved);
        if (not isSolved){
            this->isSolved = isSolved;
            cout << "Trajectory generation failed. Aborting" << endl;
            return false;
        }
        ofstream file ("QP",std::ofstream::app);
        for (uint dd = 0 ; dd < dim ; dd++) {
            file << "sol[" << dd << "]:\n" << (solSet[dd]) << endl;
            if (param.algo == ALGORITHM::END_DERIVATIVE){
                VectorX<T> df = qpFormOrig.qpBlock[dd].beq;
                VectorX<T> Phat((N+1)*this->M);
                MatrixRow<T> Afp = coeff2endDerivatives(qpFormOrig.qpBlock[0].Aeq);
                Phat << df, solSet[dd]; Phat = Afp.inverse()*Phat;
                P = scaleMatBigInv() * Phat; // polynomial (collection)
            }
            else
                P = scaleMatBigInv() * (solSet[dd]); // polynomial (collection)
            // reshaping
            for (uint m = 0; m < this->M ; m++) {
                poly_coeff_set[dd][m] =  P.segment(m * (N + 1), N + 1);
                if (isPrint)
                    cout << "Poly-coeff of dim: " << dd << " / segment: " << m  << endl << poly_coeff_set[dd][m] <<endl;
            }
        }
        this->isSolved = isSolved;
        return isSolved;
    }
    /**
     * Evalute the value of the dth dertivative trajectory at time t. If sill no solution, returing empty vector.
     * @tparam dim
     * @param t_eval evaluation time
     * @param d order of derivative
     * @return
     */
    template <typename T,size_t dim> Vector<T,dim> PolyTrajGen<T,dim>::eval(T t_eval, d_order d) {
        if (not this->isSolved){
            cout << "Evaluation request failed. Still no valid solution." << endl;
            return Vector<T,dim>();
        }
        if (t_eval < this->ts[0] or t_eval > this->ts[this->M] )
            cout << "Trajectory evaluation time out of bound. Extrapolating.. " << endl;
        uint m ;
        this->findSegInterval(t_eval,m);
        T dtm = this->ts[m+1] - this->ts[m];
        Vector<T,dim> val;
        for (uint dd = 0 ; dd < dim ; dd++)
            val(dd) = tVec(t_eval-this->ts[m],d).transpose()*poly_coeff_set[dd][m];
        return val;
    }

    /**
     * This function is permutating Aeq*p = beq  -> [Aeq ; Ap]*p (Afp * p) = [beq ; bp],
     * where Ap*p = bp is a mapping from poly-coeff to the free end-derivatives.
     * For the convenience, we consider the free end-derivative at the initial end point.
     * @tparam dim
     * @param Aeq only for one dimension (every dims are same)
     * @return [Aeq ; Ap]
     */
    template<typename T,size_t dim> MatrixRow<T> PolyTrajGen<T,dim>::coeff2endDerivatives(const trajgen::spMatrixRow<T> &Aeq) {
        uint nVar = (this->M)*(N+1); uint Nf = Aeq.rows(),Np = nVar - Nf;
        MatrixRow<T> Afp(nVar,nVar); Afp.setZero(); // collection of [Af ; Ap]
        Afp.block(0,0,Nf,nVar) = Aeq;

        vector<FixPin<T,dim>> virtualPinSet; // to generate Ap set (map to free end-derivatives)
        for (uint m = 0 ; m < this->M ; m++){
            // calculate the available dof
            VectorXi totalOrderSet(N+1); totalOrderSet.setLinSpaced(N+1,0,N);
            set<d_order> totalOrderStdSet(totalOrderSet.data(),totalOrderSet.data()+(N+1));
            vector<d_order> availbleOrder = setDiff<d_order>(totalOrderStdSet,this->fixPinOrderSet[m]);
            uint usedDOF = seg_state_set[m].getN();
            availbleOrder.resize(N+1 - usedDOF);
            for (auto freeOrder : availbleOrder )
                virtualPinSet.push_back(FixPin<T,dim>(this->ts[m],freeOrder,Vector<T,dim>()));
        }
        ConstraintMatPair<T,dim> Abp = fixPinMatSet(virtualPinSet.data(),virtualPinSet.size());
        Afp.block(Nf,0,Np,nVar) = Abp.ASet[0];
        return Afp;
    }
    /**
     * This function converts original QP problem into the QP of free end-derivatives
     * @tparam dim
     * @param QpFormOrig
     * @return
     */
    template <typename T,size_t dim> QpForm<T,dim> PolyTrajGen<T,dim>::mapQP(const trajgen::QpForm<T,dim> &QpFormOrig) {
        uint nVar = (this->M)*(N+1);
        uint Nf = QpFormOrig.qpBlock[0].Aeq.rows() , Np = (N+1)*this->M - Nf;
        uint Nineq = QpFormOrig.nineq;

        MatrixRow<T> Afp = coeff2endDerivatives(QpFormOrig.qpBlock[0].Aeq);
        MatrixRow<T> AfpInv = Afp.inverse();

        // cout << AfpInv << endl;
        MatrixRow<T> Qtmp = AfpInv.transpose() * QpFormOrig.qpBlock[0].Q.template selfadjointView<Upper>() * AfpInv;

        // cout << Qtmp << endl;

        MatrixRow<T> Qff = Qtmp.block(0,0,Nf,Nf),
        Qfp = Qtmp.block(0,Nf,Nf,Np),
        Qpf = Qtmp.block(Nf,0,Np,Nf),
        Qpp = Qtmp.block(Nf,Nf,Np,Np);
        QpForm<T,dim> QpNew(Np,0,Nineq);
        for (uint dd = 0 ; dd < dim ; dd++){
            QpNew.qpBlock[dd].Q = Qpp.sparseView();
            MatrixRow <T> Htmp = (QpFormOrig.qpBlock[dd].beq.transpose()*(Qfp+Qpf.transpose()));
            QpNew.qpBlock[dd].H = Htmp.sparseView();
            MatrixRow <T> Atmp = (QpFormOrig.qpBlock[dd].A*AfpInv.block(0,Nf,Nf+Np,Np));
            QpNew.qpBlock[dd].A = Atmp.sparseView();
            MatrixRow<T> Af =  (QpFormOrig.qpBlock[dd].A*AfpInv).block(0,0,Nineq,Nf);
            spMatrixRow<T> AfSparse = Af.sparseView();
            QpNew.qpBlock[dd].b = QpFormOrig.qpBlock[dd].b - spMatrixRow<T>(AfSparse*QpFormOrig.qpBlock[dd].beq);
        }
        return QpNew;
    }


    ///////////////////////////////
    // Optimal trajectory method //
    ///////////////////////////////

    /**
     * This function finds the nearest time index in optimtraj, given a query time
     * @tparam T
     * @tparam dim
     * @param eval_t query time
     * @param d in case of evaluation of dth order derivative, we naturally lose the last d element.
     * @return
     */
    template <typename T, size_t dim> uint OptimTrajGen<T,dim>::matchIdx(T eval_t,d_order d) {
        time_knots <T> tPntDist = this->tPnts;
        for (T & dist : tPntDist)
            dist = abs(dist-eval_t);
        int min_pos = min_element(tPntDist.begin(),tPntDist.end()) - tPntDist.begin();
        return min(min_pos,int(this->tPnts.size()-d));
    }


    template <typename T,size_t dim> OptimTrajGen<T,dim>::OptimTrajGen(
            time_knots<T> ts_,float pntDensity): TrajGen<T, dim>(ts_),pntDensity(pntDensity){
        if (ts_.size() != 2) {
            cout << "[OptimTrajGen]: time was provided more than two. "
                    "We will use only the first and last value by reintializing the base class. "
                 << endl;
            ts_[1] = ts_[ts_.size()-1]; ts_.resize(2);
            this->ts = ts_;
            this-> M = 1;
            this->fixPinSet = new vector<FixPin<T,dim>>[this->M];
            this->loosePinSet = new vector<LoosePin<T,dim>>[this->M];
            this->fixPinOrderSet = new set<d_order>[this->M];
            printf ("Initialized %d segments. \n",ts_.size()-1);
        }

        nPnt = floor((this->ts[1] - this->ts[0])*pntDensity); // total number of points
        dt = (this->ts[1] - this->ts[0])/(nPnt-1); // time interval btw points
        VectorX<T> tPntVec(nPnt); tPntVec.setLinSpaced(nPnt,this->ts[0],this->ts[1]);
        tPnts = time_knots<T>(tPntVec.data(),tPntVec.data() + nPnt);

        trajPnts = new VectorX<T>[dim];
        // initialize the trajectory
        for (uint dd =0 ; dd < dim; dd++)
            trajPnts[dd] = VectorX<T>(nPnt);
    }

    template <typename T,size_t dim> void OptimTrajGen<T,dim>::addPin(const Pin<T, dim> *pin) {
        TrajGen<T,dim>::addPin(pin);
        // do I need else ?
    }
    /**
     * For given set of loosePin, we calculate (A,b) pair s.t A x <= b for the entire dimension
     * @tparam T
     * @tparam dim
     * @param pPin
     * @param blockSize
     * @return
     */
    template<typename T,size_t dim> ConstraintMatPair<T,dim> OptimTrajGen<T,dim>::loosePinMatSet(
            const LoosePin<T,dim> * pPin,uint blockSize ){
        uint Nc = 2*blockSize; // total number of constraint
        ConstraintMatPair<T,dim> abInEq(Nc,nPnt);
        for (uint blk = 0; blk < blockSize ; blk ++ ){
            d_order d = (pPin+blk)->d;
            T t = (pPin+blk)->t;
            uint n = matchIdx(t,d); // which point should be imposed for (blk) th point
            for (uint dd = 0 ; dd < dim; dd++){
                spMatrixRow<T> pickMat(2,nPnt - d);
                pickMat.coeffRef(0,n) = 1; // upper
                pickMat.coeffRef(1,n) = -1; // lower
                spMatrixRow<T> aineq(2,nPnt); aineq = pickMat*getDiff(d);
                sparseBlockCopy<T>(&(abInEq.ASet[dd]),MatrixRow<T>(aineq),2*blk,0);
                abInEq.bSet[dd](2*blk) = (pPin+blk)->xu(dd);
                abInEq.bSet[dd](2*blk+1) = -(pPin+blk)->xl(dd);
            }
        }
        return abInEq;
    };
    /**
     * For given set of loosePin, we calculate (A,b) pair s.t Aeq x = beq for the entire dimension
     * @tparam T
     * @tparam dim
     * @param pPin
     * @param blockSize
     * @return
     */
    template <typename T, size_t dim> ConstraintMatPair<T,dim> OptimTrajGen<T,dim>::fixPinMatSet(
            const trajgen::FixPin<T, dim> *pPin, uint blockSize) {
        uint nVar = nPnt;
        uint Nc = blockSize; // only one
        ConstraintMatPair<T,dim> abEq (Nc,nVar);

        for (uint blk = 0; blk < blockSize ; blk ++){
            d_order d = (pPin+blk)->d;
            T t = (pPin+blk)->t;
            uint n = matchIdx(t,d); // which point should be imposed for (blk) th point
            for (uint dd = 0 ; dd < dim; dd++){
                spMatrixRow<T> pickMat(1,nPnt - d);
                pickMat.coeffRef(0,n) = 1; // upper
                spMatrixRow<T> aeq(1,nPnt); aeq = pickMat*getDiff(d);
                sparseBlockCopy<T>(&(abEq.ASet[dd]),aeq,blk,0);
                abEq.bSet[dd](blk) = (pPin+blk)->x(dd);
            }
        }
        return abEq;
    }

    /**
     * return the matrix Dd s.t x^(d) = Dd x. The number of points will be (nVar-d)
     * @tparam T
     * @tparam dim
     * @param d
     * @return
     */
    template<typename T,size_t dim> spMatrixRow<T> OptimTrajGen<T,dim>::getDiff(trajgen::d_order d) {
        spMatrixRow<T> D0 = spMatrixRow<T>(nPnt,nPnt); D0.setIdentity();
        if (d == 0)
            return D0;
        for (uint j = 1 ; j <= d; j++) {
            spMatrixRow<T> D(nPnt - (j), nPnt - (j - 1));
            // filling this matrix
            for (uint i = 0; i < nPnt - j; i++) {
                D.coeffRef(i,i) = -1; D.coeffRef(i,i+1) = 1;
            }
            D /= dt; D0 = D*D0;
        }
        return D0;
    }
    /**
     * construct qp problem
     * @tparam dim
     * @return
     */
    template <typename T, size_t dim> QpForm<T,dim> OptimTrajGen<T,dim>::getQPSet() {
        uint nVar = nPnt;
        uint nineq = this->getTotalNineq(); // total line of inequality
        uint neq = this->getTotalNeq();
        QpForm<T,dim> qpForm(nVar,neq,nineq); // initialize all the container

        // 1. Objective function
        spMatrixRow <T> Q(nVar,nVar);
        for (uint d = 1; d <= this->weight_mask.size() ; d++){
            spMatrixRow<T> Qd = getDiff(d).transpose()*getDiff(d);
            Q += this->weight_mask[d - 1] * Qd;
        }
        // same for every dimension
        for (uint dd = 0 ; dd < dim ; dd++)
            qpForm.qpBlock[dd].Q = Q;
        // 2. Inequality & equality
        for (uint dd = 0; dd < dim ; dd++){
            ConstraintMatPair<T,dim> Ab = loosePinMatSet(this->loosePinSet[0].data(), // in case of optimTrajGen, seg = 1
                    this->getTotalNineq()/2); // divide by 2 as the argument is number of block
            qpForm.qpBlock[dd].A = Ab.ASet[dd];
            qpForm.qpBlock[dd].b = Ab.bSet[dd].sparseView();

            ConstraintMatPair<T,dim> Abeq = fixPinMatSet(this->fixPinSet[0].data(),
                                                         this->getTotalNeq()); // divide by 2 as the argument is number of block

            qpForm.qpBlock[dd].Aeq = Abeq.ASet[dd];
            qpForm.qpBlock[dd].beq = Abeq.bSet[dd].sparseView();

        }

        return qpForm;
    }

    /**
     * solve the quadratic programming
     * @tparam T
     * @tparam dim
     * @param isPrint
     * @return
     */
    template <typename T, size_t dim> bool OptimTrajGen<T,dim>::solve(bool isPrint) {
        if (this->getTotalNeq() == 0 and this->getTotalNineq() == 0) {
            cout << "No constraints exist. Aborting. " << endl;
            return false;
        }
        QpForm<T, dim> qpForm = getQPSet();
        qpForm.write();

        vector<VectorX<T>> solSet = qpForm.getQpSolSet(this->isSolved);
        if (not this->isSolved) {
            cout << "Trajectory generation failed. Aborting" << endl;
            return false;
        }
        for (uint dd = 0; dd < dim ; dd++)
            trajPnts[dd] = solSet[dd];
        return true;
    }

    /**
     * Evalute the value of the dth dertivative trajectory at time t. If sill no solution, returing empty vector.
     * @tparam dim
     * @param t_eval evaluation time
     * @param d order of derivative
     * @return
     */
     template <typename T, size_t dim > Vector<T,dim> OptimTrajGen<T,dim>::eval(T t_eval, d_order d) {
         if (not this->isSolved){
            cout << "Evaluation request failed. Still no valid solution." << endl;
            return Vector<T,dim>();
         }
        // warning  extra-polation turn on
        if (t_eval < this->ts[0] or t_eval > this->ts[this->M] )
            cout << "Trajectory evaluation time out of bound. Extrapolating.. " << endl;

        Vector<T,dim> val;
        VectorX<T> time_knots_vec = Map<Matrix<T,-1,1>>(tPnts.data(),tPnts.size(),1);
        for (uint dd = 0; dd < dim ; dd++) {
            val(dd) = interpolate<T>(time_knots_vec.segment(0,nPnt-d), getDiff(d)*trajPnts[dd], t_eval,true);
        }
        return val;
     }

} // namespace trajgen
# endif
