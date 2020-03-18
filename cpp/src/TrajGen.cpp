//
// Created by jbs on 20. 3. 17..
//
#include <traj_gen2/TrajGen.hpp>

void sparseBlockCopy(SparseMatrix<float,RowMajor> *targetMat, SparseMatrix<float,RowMajor> inMat, long startRow, long startCol) {
    for (int k = 0; k < inMat.rows(); ++k)
        for (SparseMatrix<float,RowMajor>::InnerIterator it(inMat, k); it; ++it) { // we use sparsity
            long insertR = it.row() + startRow;
            long insertC = it.col() + startCol;
            targetMat->coeffRef(insertR, insertC) = it.value();
        }
}

void sparseBlockCopy(SparseMatrix<float,RowMajor> *targetMat, Matrix<float,-1,-1,RowMajor> inMat, long startRow, long startCol) {
    for (int k = 0; k < inMat.rows(); ++k)
        for (Matrix<float,-1,-1,RowMajor>::InnerIterator it(inMat, k); it; ++it) { // we use sparsity
            long insertR = it.row() + startRow;
            long insertC = it.col() + startCol;
            targetMat->coeffRef(insertR, insertC) = it.value();
        }
}

namespace trajgen{

    QpBlock::QpBlock(uint nVar, uint neq, uint nineq) {
        Q = spMatrixXf(nVar, nVar);
        H = spMatrixXf(1, nVar);
        A = spMatrixXf(nineq, nVar);
        b = spMatrixXf(nineq, 1);
        Aeq = spMatrixXf(neq, nVar);
        beq = spMatrixXf(neq, 1);
    }

    VectorXf QpBlock::getQpSol(bool & isSolved) {
        uint nVar = Q.rows();
        int N_ineq_const = A.rows();
        int N_eq_const = Aeq.rows();
        int N_const = N_ineq_const + N_eq_const;
        // convention follows the qpOASES
        qpOASES::QProblem qp_obj(nVar, N_const, qpOASES::HST_SEMIDEF);
        Matrix<double, -1, -1, RowMajor> Hmat(2 * Q.cast<double>());
        Matrix<double, 1, -1, RowMajor> gmat(H.cast<double>());
        Matrix<double, -1, -1, RowMajor> Amat(N_const, nVar);
        Amat << MatrixXd(A.cast<double>()), MatrixXd(Aeq.cast<double>());
        Matrix<double, -1, -1, RowMajor> ubAmat(N_const, 1);
        ubAmat << MatrixXd(b.cast<double>()), MatrixXd(beq.cast<double>());
        MatrixXd MinusInf(N_ineq_const, 1);
        MinusInf=MinusInf.setConstant(-99999);
        Matrix<double, -1, -1, RowMajor> lbAmat(N_const, 1);
        lbAmat << MinusInf, MatrixXd(beq.cast<double>());

        qpOASES::real_t *Hqp = Hmat.data();
        qpOASES::real_t *gqp = gmat.data();
        qpOASES::real_t *Aqp = Amat.data();
        qpOASES::real_t *lbAqp = lbAmat.data();
        qpOASES::real_t *ubAqp = ubAmat.data();

        qpOASES::Options options;
        options.printLevel = qpOASES::PL_HIGH;
        qp_obj.setOptions(options);
        qpOASES::int_t nWSR = 2000;
        qp_obj.init(Hqp, gqp, Aqp, NULL, NULL, lbAqp, ubAqp, nWSR);
        if(qp_obj.isInfeasible()){
            cout<<"[QP solver] warning: problem is infeasible. "<<endl;
        }

        cout << "Acutual nWSR: " << nWSR << " / Original: " << 2000 << endl;

        qpOASES::real_t xOpt[nVar];
        qp_obj.getPrimalSolution(xOpt);
        isSolved = qp_obj.isSolved();
        if (isSolved)
            cout<<"[QP solver] success.  "<<endl;

        return Map<Matrix<double, -1, 1>>(xOpt, nVar, 1).cast<float>();
    }


}