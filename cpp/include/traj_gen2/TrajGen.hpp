# ifndef TRAJ_GEN
# define TRAJ_GEN
//////////////////////////////////////////////////////////////////////////////////
// TODO : Preventing double imposed pins (t,d same, X different)
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

using namespace Eigen;
using namespace std;

/**
 * Insert inMat to targetMat.block(startRow,startCol,rowSize,colSize).
 * @param targetMat
 * @param inMat
 * @param startRow
 * @param startCol
 * @param rowSize
 * @param colSize
 */
void sparseBlockCopy(SparseMatrix<float,RowMajor> *targetMat, SparseMatrix<float,RowMajor> inMat, long startRow, long startCol) ;
void sparseBlockCopy(SparseMatrix<float,RowMajor> *targetMat, Matrix<float,-1,-1,RowMajor> inMat, long startRow, long startCol) ;

namespace trajgen {
    typedef unsigned int uint;
    template<size_t Size> using Vector = Eigen::Matrix<float, Size, 1>;
    typedef uint d_order; // derivative order
    typedef uint p_order; // polynomial order
    typedef vector<float> time_knots;
    typedef SparseMatrix<float,RowMajor> spMatrixXf;
    using PolyCoeff = Eigen::Matrix<float, -1, 1>;

    /////////////////////////////////////////////////
    // PIN for equality or inequality constriants  //
    /////////////////////////////////////////////////

    // *NOTE*
    // We use vector container for unknown size data stack
    // while array type is used for a container whose size can be determined at constructor

    enum PIN_TYPE {
        FIX_PIN = 0, // equality constraint
        LOOSE_PIN = 1 // inequality constraint
    };

    template<size_t dim>
    struct Pin {
        float t; // imposed time
        uint d; // imposed order of derivative          
        Pin(float t_, d_order d_) : t(t_), d(d_) {};

        virtual PIN_TYPE getType() const = 0;
    };

    template<size_t dim>
    struct FixPin : public Pin<dim> {
        Vector<dim> x; // waypoint 
        FixPin(float t_, uint d_, Vector<dim> x_) : Pin<dim>(t_, d_), x(x_) {};
        PIN_TYPE getType() const { return PIN_TYPE::FIX_PIN; }
    };

    template<size_t dim>
    struct LoosePin : public Pin<dim> {
        Vector<dim> xl; // lower bound
        Vector<dim> xu; // upper bound
        LoosePin(float t_, uint d_, Vector<dim> xl_, Vector<dim> xu_) : Pin<dim>(t_, d_), xl(xl_), xu(xu_) {};
        PIN_TYPE getType() const { return PIN_TYPE::LOOSE_PIN; }
    };

    /////////////////////////////////////
    // Quadratic programming structure //
    /////////////////////////////////////

    template<size_t dim>
    struct ConstraintMatPair {
        vector<spMatrixXf> ASet;
        vector<MatrixXf> bSet;

        ConstraintMatPair() {
            ASet.resize(dim), bSet.resize(dim);
        }

        void initialize(uint Nc, uint nVar) {
            for (spMatrixXf &sp : ASet)
                sp = spMatrixXf(Nc, nVar);
            for (MatrixXf &mat : bSet)
                mat = MatrixXf::Zero(Nc, 1);
        }
        ConstraintMatPair(uint Nc, uint nVar) {
            ASet.resize(dim), bSet.resize(dim);
            this->initialize(Nc, nVar);
        }
    };

    // p'Q p + H p : interact with qpOASES
    struct QpBlock {
        spMatrixXf Q;
        spMatrixXf H;
        spMatrixXf A;
        spMatrixXf b;
        spMatrixXf Aeq;
        spMatrixXf beq;
        QpBlock(uint nVar, uint neq, uint nineq);
        QpBlock() {};
        VectorXf getQpSol(bool & isSolved);
    };

    template<size_t dim>
    struct QpForm {
        QpBlock *qpBlock;
        QpForm(uint nVar, uint neq, uint nineq) {
            qpBlock = new QpBlock[dim];
            for (uint d = 0; d < dim; d++)
                qpBlock[d] = QpBlock(nVar, neq, nineq);
        };
        void write();
        vector<VectorXf> getQpSolSet(bool & isSolved);
    };

    /////////////////////////////
    // TrajGen as a base class //
    /////////////////////////////

    template<size_t dim>
    class TrajGen {

    protected:
        uint M; // number of segment
        bool isSolved = false; // solve flag
        time_knots ts; // knots

        vector<FixPin<dim>> *fixPinSet;  // equality constraints set of M segment. (In case of optimTrajGen, M = 1)
        vector<LoosePin<dim>> *loosePinSet; // inequality constraints set of ()
        set<d_order> *fixPinOrderSet;
        VectorXf weight_mask; // high order derivative penalty weights

        // Subroutine functions
        void findSegInterval(float t, uint &m);
        void findSegInterval(float t, uint &m, float &tau);

        virtual ConstraintMatPair<dim> fixPinMatSet(const FixPin<dim> *pPin,uint blockSize = 1) = 0;
        virtual ConstraintMatPair<dim> loosePinMatSet(const LoosePin<dim> *pPin,uint blockSize = 1) = 0;
        virtual QpForm<dim> getQPSet() = 0; // create qp problem with current information
        uint getTotalNineq();
        virtual uint getTotalNeq();

    public:
        TrajGen(time_knots ts_);
        void addPinSet(const vector<Pin<dim> *> &pinPtrSet);
        void setDerivativeObj(VectorXf weights);
        virtual void addPin(const Pin<dim> *pin);
        virtual Vector<dim> eval(float t_eval, d_order d) = 0;
        virtual bool solve(bool verbose) = 0;
    };

    ///////////////////////////////////////////
    // PolyTrajGen for piecewise polynomials //
    ///////////////////////////////////////////

    enum ALGORITHM{
        POLY_COEFF = 0,
        END_DERIVATIVE = 1
    };         

    // PolyParameters to define the behavior of polynomial
    struct PolyParam{
        p_order poly_order = 4;
        d_order max_conti_order = 2;
        ALGORITHM algo = ALGORITHM::POLY_COEFF;
        PolyParam() { printf("No default parameters given. Default values are used\n");};     
        PolyParam(p_order N,d_order max_conti,ALGORITHM algo): poly_order(N),max_conti_order(max_conti),algo(algo) {};
    };
    
    // State of the polynomial segment. 
    struct PolyState{
        uint Nc = 0; // order of continuity             
        uint Nf = 0; // number of fixed pins
        uint getN() {return Nc+Nf;};
    };

    template<size_t dim>
    class PolyTrajGen : public TrajGen<dim>{
        
        private:
            PolyParam param;
            PolyState* seg_state_set;
            PolyCoeff** poly_coeff_set; // [dim][M]
            p_order N; // poly order

            // subroutine functions
            float B(p_order n,d_order d);
            VectorXf tVec(float t,d_order d);
            spMatrixXf scaleMat(float dt);
            spMatrixXf scaleMatBig();
            spMatrixXf scaleMatBigInv();

            // objective
            spMatrixXf IntDerSquard(d_order d);
            // constraints
            ConstraintMatPair<dim> contiMat(uint m, d_order dmax);
            ConstraintMatPair<dim> fixPinMatSet(const FixPin<dim> * pPin,uint blockSize = 1);
            ConstraintMatPair<dim> loosePinMatSet(const LoosePin<dim> * pPin,uint blockSize = 1);
            uint getTotalNeq();
            // qp
            QpForm<dim> getQPSet();
    public:
            PolyTrajGen(time_knots ts_,PolyParam param_) ;
            void addPin(const Pin<dim>* pin);
            Vector<dim> eval(float t_eval, d_order d);
            bool solve(bool verbose);


    };


    ///////////////////////////////////////////////////////////////////////////////////////

           // FUNCTION DEFINITIONS (as we use template, better to include defs here) //

    ///////////////////////////////////////////////////////////////////////////////////////



    /////////////////////////////////////
    // Quadratic programming structure //
    /////////////////////////////////////


    /**
     * write current problem
     */
    template<size_t dim> void QpForm<dim>::write() {
        ofstream file ("QP");
        file << "Q:" << "\n" << MatrixXf(qpBlock[0].Q) << endl;
        file << "H:" << "\n" << MatrixXf(qpBlock[0].H) << endl;
        file << "A:" << "\n" << MatrixXf(qpBlock[0].A) << endl;
        for (uint dd = 0 ; dd < dim ; dd++)
            file << "b[" << dd << "]:\n" << MatrixXf(qpBlock[dd].b) << endl;

        file << "Aeq:" << "\n" << MatrixXf(qpBlock[0].Aeq) << endl;
        for (uint dd = 0 ; dd < dim ; dd++)
            file << "beq[" << dd << "]:\n" << MatrixXf(qpBlock[dd].beq) << endl;
        file.close();
    }

    template <size_t dim> vector<VectorXf> QpForm<dim>::getQpSolSet(bool & isSolved) {
        vector<VectorXf> qpSolSet(dim);
        isSolved = true;
        for (uint dd = 0; dd < dim ; dd++) {
            qpSolSet[dd] = qpBlock[dd].getQpSol(isSolved);
            if (not isSolved)
                break;
        }
        return qpSolSet;
    }

    /////////////////////////////
    // TrajGen as a base class //
    /////////////////////////////
    template <size_t dim> TrajGen<dim>::TrajGen(time_knots ts_): ts(ts_){
        // pinset per segment
        M = ts.size()-1; // # of segment = length(knots) - 1
        this->fixPinSet = new vector<FixPin<dim>>[M];
        this->loosePinSet = new vector<LoosePin<dim>>[M];
        fixPinOrderSet = new set<d_order>[M];
        printf ("Initialized %d segments. \n",ts_.size()-1);
    }

    /**
     * Find a segment index (0,1,...,M-1) which contains t (ts[m]<= t < ts[m+1]  || t< ts[1] || ts[M-1]<= t)
     * @tparam dim
     * @param t query time
     * @param m segment index
     */
    template<size_t dim> void TrajGen<dim>::findSegInterval(float t,uint& m){
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
    template <size_t dim> void TrajGen<dim>::findSegInterval(float t, uint &m, float &tau) {
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
    template<size_t dim> void TrajGen<dim>::addPin(const Pin<dim> *pin){
        uint m;
        this->findSegInterval(pin->t,m); // which segment to add the pin
        if (pin->getType() == PIN_TYPE::FIX_PIN)
            fixPinSet[m].push_back(*(static_cast<const FixPin<dim>*>(pin)));
        else
            loosePinSet[m].push_back(*(static_cast<const LoosePin<dim>*>(pin)));
    }

    template<size_t dim> void TrajGen<dim>::addPinSet(const vector<Pin<dim>*>& pinPtrSet){
        for (auto it = pinPtrSet.begin() ; it < pinPtrSet.end() ; it++)
            addPin(*it);
    }

    template<size_t dim> void TrajGen<dim>::setDerivativeObj(VectorXf weights){
        weight_mask = weights;
    }
    /**
     *     get total number of inequality condition. Per a loosePin, two inequalities
     */
    template<size_t dim> uint TrajGen<dim>::getTotalNineq() {
        uint nineq = 0;
        for (uint m = 0 ; m < M ; m++)
            nineq+= 2*loosePinSet[m].size();
        return nineq;
    }
    template<size_t dim> uint TrajGen<dim>::getTotalNeq() {
        uint neq = 0;
        for (uint m = 0 ; m < M ; m++)
            neq+= fixPinSet[m].size();
        return neq;
    }


    ///////////////////////////////////////////
    // PolyTrajGen for piecewise polynomials //
    ///////////////////////////////////////////

    template<size_t dim> PolyTrajGen<dim>::PolyTrajGen(time_knots ts_,PolyParam param_)  : TrajGen<dim>(ts_), param(param_),N(param_.poly_order){

        seg_state_set = new PolyState[this->M];

        // polycoeff segment / dimension
        poly_coeff_set = new PolyCoeff*[dim];
        for (int dd = 0;dd <dim ; dd++)
            poly_coeff_set[dd] = new PolyCoeff[this->M];


        // set the default Nc (except the last segment )
        for (uint m = 0 ; m < (this->M - 1); m++)
            seg_state_set[m].Nc = param_.max_conti_order+1;

    }


    template<size_t dim> void PolyTrajGen<dim>::addPin(const Pin<dim>* pin){
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
                TrajGen<dim>::addPin(pin);
                seg_state_set[m].Nf+= 1;
                if (seg_state_set[m].Nf +seg_state_set[m].Nc > N+1 ) // if this pin addition overflow the total # of dof
                {
                    seg_state_set[m].Nc -= 1;
                    printf ("Fix pin reduced the continuity order (seg : %d, Nf=%d / Nc=%d).\n",seg_state_set[m].Nf,seg_state_set[m].Nc);
                }
                this->fixPinOrderSet[m].insert(pin->d);
            }
        else
            TrajGen<dim>::addPin(pin);
    }

    /**
     * The nth order coefficient of time vector ([1 t t^2 ... t^N]) when differentiated d times.
     * @tparam dim
     * @param n : target order
     * @param d : order of derivative
     * @return
     */
    template <size_t dim> float PolyTrajGen<dim>::B(p_order n,d_order d){
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
    template <size_t dim> VectorXf PolyTrajGen<dim>::tVec(float t,d_order d){
        VectorXf vec(this->N+1); vec.setZero();
        for (int n = d ; n < N+1 ; n ++)
            vec(n) = B(n,d)*pow(t,n-d);
        return vec;
    }

    template <size_t dim> spMatrixXf PolyTrajGen<dim>::scaleMat(float dt){
        spMatrixXf mat(N+1,N+1);

        vector<Triplet<float>> tripletList;
        for (int i = 0 ; i < N+1 ; i++)
            tripletList.emplace_back(i,i,pow(dt,i));
        mat.setFromTriplets(tripletList.begin(),tripletList.end());
        return mat;
    }
    template <size_t dim> spMatrixXf PolyTrajGen<dim>::scaleMatBig() {
        spMatrixXf mat(this->M*(N+1),this->M*(N+1));
        vector<Triplet<float>> tripletList;
        for (int m = 0 ; m < this->M ; m++)
            for (int i = 0 ; i < N+1 ; i++)
                tripletList.emplace_back( (N+1)*m + i  ,(N+1)*m + i, pow(this->ts[m+1] - this->ts[m],i));
        mat.setFromTriplets(tripletList.begin(),tripletList.end());
        return mat;
    }

    template <size_t dim> spMatrixXf PolyTrajGen<dim>::scaleMatBigInv() {
        spMatrixXf mat(this->M*(N+1),this->M*(N+1));
        vector<Triplet<float>> tripletList;
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
    template <size_t dim> spMatrixXf PolyTrajGen<dim>::IntDerSquard(trajgen::d_order d) {
        spMatrixXf Q(N+1,N+1);
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
     *
     * @tparam dim
     * @param pPin pointer to the first element of the array pin
     * @param blockSize how many elements to be considered
     * @return
     */
    template <size_t dim> ConstraintMatPair<dim> PolyTrajGen<dim>::fixPinMatSet(const FixPin<dim> *pPin,uint blockSize){
        uint nVar = this->M * (N+1);
        uint Nc = blockSize; // only one
        ConstraintMatPair<dim> abEq (Nc,nVar);

        for (uint blk = 0; blk < blockSize ; blk++) {
            uint m; float tau;
            this->findSegInterval((pPin+blk)->t, m, tau);
            uint idxStart = m * (N + 1);
            float dt = this->ts[m + 1] - this->ts[m];
            d_order d = (pPin+blk)->d;
            for (uint dd = 0; dd < dim; dd++) {
                sparseBlockCopy(&(abEq.ASet[dd]),tVec(tau,d).transpose()/pow(dt,d),blk,idxStart);
                abEq.bSet[dd](blk) = (pPin + blk)->x(dd);
            }
        }
        return abEq;
    }


    template <size_t dim> ConstraintMatPair<dim> PolyTrajGen<dim>::loosePinMatSet(const LoosePin<dim> *pPin,uint blockSize){
        uint nVar = this->M * (N+1);
        uint Nc = 2*blockSize;
        ConstraintMatPair<dim> abInEq (Nc,nVar);
       for (uint blk = 0 ; blk < blockSize ; blk ++) {
           uint m; float tau;
           this->findSegInterval((pPin+blk)->t, m, tau);
           uint idxStart = m * (N + 1);
           float dt = this->ts[m + 1] - this->ts[m];
           d_order d = (pPin+blk)->d;
           for (uint dd = 0; dd < dim; dd++) {

               sparseBlockCopy(&(abInEq.ASet[dd]),tVec(tau,d).transpose()/pow(dt,d),2*blk,idxStart);
               sparseBlockCopy(&(abInEq.ASet[dd]),-tVec(tau,d).transpose()/pow(dt,d),2*blk+1,idxStart);

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
     * @param dmax : we impose  dmax continuity (0,1,...,dmax)
     * @return
     */
    template <size_t dim> ConstraintMatPair<dim> PolyTrajGen<dim>::contiMat(uint m, trajgen::d_order dmax) {

        uint nVar = this->M *(N+1);

        if (m == this->M-1)
            return ConstraintMatPair<dim>(0,nVar);

        uint Nc = dmax + 1;
        ConstraintMatPair<dim> abEq(Nc,nVar);
        uint idxStart = m*(N+1);
        float dt1 = this->ts[m+1] - this->ts[m];
        float dt2 = this->ts[m+2] - this->ts[m+1];

        for (uint dd = 0; dd < dim ; dd++)
            for (d_order d = 0; d <= dmax; d++) {
                sparseBlockCopy(&(abEq.ASet[dd]),tVec(1, d).transpose() / pow(dt1, d),d,idxStart);
                sparseBlockCopy(&(abEq.ASet[dd]),-tVec(0, d).transpose() / pow(dt2, d),d,idxStart + (N+1));

                abEq.bSet[dd](d) = 0;
            }

        return abEq;
    }
    /**
     * Get the total number of equality constraint (sum (Nc+Nf) for m = 0,,,M-1)
     * @tparam dim
     * @return
     */
    template<size_t dim> uint PolyTrajGen<dim>::getTotalNeq() {
        auto lambda = [&] (uint s1,PolyState s2){return s1+s2.getN();};
        return accumulate(seg_state_set,seg_state_set+this->M,0,lambda);
    };


    /**
     * construct qp problem
     * @tparam dim
     * @return
     */
    template<size_t dim> QpForm<dim> PolyTrajGen<dim>::getQPSet() {

        uint nVar = (this->M)*(N+1);
        uint nineq = this->getTotalNineq(); // total line of inequality
        uint neq = this->getTotalNeq();
        QpForm<dim> qpForm(nVar,neq,nineq); // initialize all the container

        // 1. Objective function
        spMatrixXf Q(nVar,nVar);
        for (uint d = 1; d <= this->weight_mask.size(); d++) { // dth order derivative
            spMatrixXf Qd(nVar, nVar);
            for (uint m = 0; m < this->M; m++) { // per segment, plug in the Q
                float dt = this->ts[m + 1] - this->ts[m];
                sparseBlockCopy(&Qd,MatrixXf(IntDerSquard(d)/pow(dt,2*d-1)),m*(N+1),m*(N+1));
            }
            Q += this->weight_mask[d - 1] * Qd;
        }

        for (uint dd = 0 ; dd < dim ; dd++)
            qpForm.qpBlock[dd].Q = Q;

        uint Idx1 = 0,Idx2 = 0;
        for (uint m = 0 ; m < this->M ; m++){ // per segment
            // 2. Ineq constraint for this segment
            uint Nineq = this->loosePinSet[m].size();
            ConstraintMatPair<dim> Ab = loosePinMatSet((this->loosePinSet[m]).data(),this->loosePinSet[m].size());
            for (uint dd = 0 ; dd < dim ; dd++) { // per dimension
                sparseBlockCopy(&(qpForm.qpBlock[dd].A),(Ab.ASet[dd]),Idx1,0);
                sparseBlockCopy(&(qpForm.qpBlock[dd].b),(Ab.bSet[dd]),Idx1,0);
            }
            Idx1+= 2*Nineq;

            // 3. Eq constraint for this segment
            ConstraintMatPair<dim> AbFix = fixPinMatSet((this->fixPinSet[m]).data(),this->fixPinSet[m].size());
            ConstraintMatPair<dim> AbConti = contiMat(m,seg_state_set[m].Nc-1);
            uint Neq = seg_state_set[m].getN(), Nf = seg_state_set[m].Nf, Nc = seg_state_set[m].Nc;
            for (uint dd = 0 ; dd < dim ; dd++) { // inserting A,b

                sparseBlockCopy(&(qpForm.qpBlock[dd].Aeq),(AbFix.ASet[dd]),Idx2,0) ;
                sparseBlockCopy(&(qpForm.qpBlock[dd].Aeq),(AbConti.ASet[dd]),Idx2+Nf,0);
                sparseBlockCopy(&(qpForm.qpBlock[dd].beq),(AbFix.bSet[dd]),Idx2,0) ;
                sparseBlockCopy(&(qpForm.qpBlock[dd].beq),(AbConti.bSet[dd]),Idx2+Nf,0);

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
    template <size_t dim > bool PolyTrajGen<dim>::solve(bool isPrint){
        // Prepare QP
        QpForm<dim> qpForm = getQPSet();
        qpForm.write();

        // Solve
        bool isSolved;
        vector<VectorXf> solSet = qpForm.getQpSolSet(isSolved);
        ofstream file ("QP",std::ofstream::app);
        for (uint dd = 0 ; dd < dim ; dd++) {
            file << "sol[" << dd << "]:\n" << (solSet[dd]) << endl;
            VectorXf P = scaleMatBigInv() * (solSet[dd]); // polynomial (collection)
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
     * Evalute the value of the dth dertivative trajectory at time t.
     * @tparam dim
     * @param t_eval evaluation time
     * @param d order of derivative
     * @return
     */
    template <size_t dim> Vector<dim> PolyTrajGen<dim>::eval(float t_eval, d_order d) {
        if (t_eval < this->ts[0] or t_eval > this->ts[this->M] )
            cout << "Trajectory evaluation time out of bound. Extrapolating.. " << endl;
        uint m ;
        this->findSegInterval(t_eval,m);
        float dtm = this->ts[m+1] - this->ts[m];
        Vector<dim> val;
        for (uint dd = 0 ; dd < dim ; dd++)
            val(dd) = tVec(t_eval-this->ts[m],d).transpose()*poly_coeff_set[dd][m];
        return val;
    }



} // namespace trajgen
# endif
