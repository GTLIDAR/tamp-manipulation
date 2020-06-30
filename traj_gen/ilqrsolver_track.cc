#include "drake/traj_gen/ilqrsolver_track.h"

/* Debug */
#include <iostream>
using namespace std;
/* */

using namespace Eigen;
using Eigen::VectorXd;

namespace drake {
namespace traj_gen {
namespace kuka_iiwa_arm {

ILQRSolver_TRK::ILQRSolver_TRK(KukaArm_TRK& iiwaDynamicModel, CostFunctionKukaArm_TRK& iiwaCostFunction, bool fullDDP, bool QPBox)
{
    //TRACE("initialize dynamic model and cost function\n");
    dynamicModel = &iiwaDynamicModel;
    costFunction = &iiwaCostFunction;
    stateNb = iiwaDynamicModel.getStateNb();
    commandNb = iiwaDynamicModel.getCommandNb();
    enableQPBox = QPBox;
    enableFullDDP = fullDDP;

    if(enableQPBox) TRACE("Box QP is enabled\n");
    else TRACE("Box QP is disabled\n");

    if(enableFullDDP) TRACE("Full DDP is enabled\n");
    else TRACE("Full DDP is disabled\n");

    // if(QPBox)
    // {
    //     qp = new QProblemB(commandNb);
    //     Options iiwaOptions;
    //     iiwaOptions.printLevel = PL_LOW;
    //     iiwaOptions.enableRegularisation = BT_TRUE;
    //     iiwaOptions.initialStatusBounds = ST_INACTIVE;
    //     iiwaOptions.numRefinementSteps = 1;
    //     iiwaOptions.enableCholeskyRefactorisation = 1;
    //     qp->setOptions(iiwaOptions);

    //     xOpt = new real_t[commandNb];
    //     lowerCommandBounds = iiwaDynamicModel.getLowerCommandBounds();
    //     upperCommandBounds = iiwaDynamicModel.getUpperCommandBounds();
    // }

    //tOptSet Op = INIT_OPTSET;
}

void ILQRSolver_TRK::firstInitSolver(stateVec_t& iiwaxInit, stateVec_t& iiwaxgoal, stateVecTab_t& x_bar, commandVecTab_t& u_bar, 
        commandVecTab_t initialTorque, unsigned int& iiwaN, double& iiwadt, unsigned int& iiwamax_iter, double& iiwatolFun, double& iiwatolGrad)
{
    // TODO: double check opt params
    xInit = iiwaxInit; // removed iiwaxgoal. Double check whether this makes sense.
    xgoal = iiwaxgoal;
    xList_bar = x_bar;
    uList_bar = u_bar;
    initCommand = initialTorque;
    N = iiwaN;
    dt = iiwadt;
    
    //TRACE("initialize option parameters\n");
    //Op = INIT_OPTSET;
    standardizeParameters(&Op);
    Op.xInit = iiwaxInit;
    Op.n_hor = N;
    Op.tolFun = iiwatolFun;
    Op.tolGrad = iiwatolGrad;
    Op.max_iter = iiwamax_iter;
    Op.time_backward.resize(Op.max_iter);
    Op.time_backward.setZero();
    Op.time_forward.resize(Op.max_iter);
    Op.time_forward.setZero();
    Op.time_derivative.resize(Op.max_iter);
    Op.time_derivative.setZero();

    xList.resize(N+1);
    uList.resize(N);
    uListFull.resize(N+1);
    uList_bar_Full.resize(N+1);
    updatedxList.resize(N+1);
    updateduList.resize(N);
    costList.resize(N+1);
    costListNew.resize(N+1);
    kList.resize(N);
    KList.resize(N);
    FList.resize(N+1);
    Vx.resize(N+1);
    Vxx.resize(N+1);
    
    for(unsigned int i=0;i<N;i++){
        xList[i].setZero();
        uList[i].setZero();
        uListFull[i].setZero();
        updatedxList[i].setZero();
        updateduList[i].setZero();
        costList[i] = 0;
        costListNew[i] = 0;
        kList[i].setZero();
        KList[i].setZero();
        FList[i].setZero();    
        Vx[i].setZero();
        Vxx[i].setZero();
    }
    xList[N].setZero();
    uListFull[N].setZero();
    uList_bar_Full[N].setZero();
    updatedxList[N].setZero();
    costList[N] = 0;
    costListNew[N] = 0;
    FList[N].setZero();
    Vx[N].setZero();
    Vxx[N].setZero();
    
    k.setZero();
    K.setZero();
    dV.setZero();

    // parameters for line search
    Op.alphaList.resize(11);
    Op.alphaList << 1.0, 0.5012, 0.2512, 0.1259, 0.0631, 0.0316, 0.0158, 0.0079, 0.0040, 0.0020, 0.0010;

    debugging_print = 0;
}

void ILQRSolver_TRK::solveTrajectory()
{
    //==============
    // Checked!!v
    //==============
    initializeTraj();

    if(Op.debug_level > 0) TRACE("\n=========== begin iLQR ===========\n");
    // for(unsigned int i=0;i<=N;i++)
    // {
    //   cout << "init traj xList[" << i << "]:" << updatedxList[i].transpose() << endl;
    // }

    Op.lambda = Op.lambdaInit;
    Op.dlambda = Op.dlambdaInit;
    
    // TODO: update multipliers
    //update_multipliers(Op, 1);

    for(iter=0;iter<Op.max_iter;iter++)
    {
        //==============
        // Check TODO
        //==============
        // TRACE("STEP 1: differentiate dynamics and cost along new trajectory\n");
        if(newDeriv){
            int nargout = 7;//fx,fu,cx,cu,cxx,cxu,cuu
            for(unsigned int i=0;i<u_NAN.size();i++)
                u_NAN(i,0) = sqrt(-1.0); // control vector = Nan for last time step
            
            for(unsigned int i=0;i<uList.size();i++) {
                uListFull[i] = uList[i];
                uList_bar_Full[i] = uList_bar[i];
                // cout << "UlistFULL " << i << endl;
                // cout << uListFull[i] << endl;
            }
            uListFull[uList.size()] = u_NAN;
            uList_bar_Full[uList.size()] = u_NAN;
            gettimeofday(&tbegin_time_deriv,NULL);
            // FList is empty here on the first interation
            // initial x, u, cost is passed in
            dynamicModel->kuka_arm_dyn_cst_ilqr(nargout, xList, uListFull, FList, xList_bar, uList_bar_Full, costFunction);
            //dynamicModel->kuka_arm_dyn_cst(nargout, dt, xList, uListFull, xgoal, FList, costFunction->getcx(), costFunction->getcu(), costFunction->getcxx(), costFunction->getcux(), costFunction->getcuu(), costFunction->getc());
            gettimeofday(&tend_time_deriv,NULL);
            Op.time_derivative(iter) = (static_cast<double>(1000*(tend_time_deriv.tv_sec-tbegin_time_deriv.tv_sec)+((tend_time_deriv.tv_usec-tbegin_time_deriv.tv_usec)/1000)))/1000.0;

            newDeriv = 0;
        }

        //==============
        // Check TODO
        //==============
        // TRACE("====== STEP 2: backward pass, compute optimal control law and cost-to-go\n");
        backPassDone = 0;
        while(!backPassDone){

            gettimeofday(&tbegin_time_bwd,NULL);
            doBackwardPass();
            gettimeofday(&tend_time_bwd,NULL);
            Op.time_backward(iter) = (static_cast<double>(1000*(tend_time_bwd.tv_sec-tbegin_time_bwd.tv_sec)+((tend_time_bwd.tv_usec-tbegin_time_bwd.tv_usec)/1000)))/1000.0;

            //TRACE("handle Cholesky failure case");
            if(diverge){
                if(Op.debug_level > 1) printf("Cholesky failed at timestep %d.\n",diverge);
                Op.dlambda   = max(Op.dlambda * Op.lambdaFactor, Op.lambdaFactor);
                Op.lambda    = max(Op.lambda * Op.dlambda, Op.lambdaMin);
                if(Op.lambda > Op.lambdaMax) break;

                    continue;
            }
            backPassDone = 1;
        }

        // check for termination due to small gradient
        // TODO: add constraint tolerance check
        if(Op.g_norm < Op.tolGrad && Op.lambda < 1e-5){
            Op.dlambda= min(Op.dlambda / Op.lambdaFactor, 1.0/Op.lambdaFactor);
            Op.lambda= Op.lambda * Op.dlambda * (Op.lambda > Op.lambdaMin);
            if(Op.debug_level>=1){
                TRACE(("\nSUCCESS: gradient norm < tolGrad\n"));
            }
            break;
        }

        //====== STEP 3: line-search to find new control sequence, trajectory, cost
        fwdPassDone = 0;
        if(backPassDone){
            gettimeofday(&tbegin_time_fwd,NULL);
            //only implement serial backtracking line-search
            for(int alpha_index = 0; alpha_index < Op.alphaList.size(); alpha_index++){
                alpha = Op.alphaList[alpha_index];
                doForwardPass();
                Op.dcost = accumulate(costList.begin(), costList.end(), 0.0) - accumulate(costListNew.begin(), costListNew.end(), 0.0);
                Op.expected = -alpha*(dV(0) + alpha*dV(1));

                // cout << "precost " << accumulate(costList.begin(), costList.end(), 0.0) << endl;
                // cout << "newcost " << accumulate(costListNew.begin(), costListNew.end(), 0.0) << endl;
                // cout << "Dv0 " << dV(0) << " || Dv1 " << dV(1) << endl;
                // cout << "dcost " << Op.dcost << endl;
                // cout << "dexp " << Op.expected << endl;

                double z;
                if(Op.expected > 0) {
                    z = Op.dcost/Op.expected;
                }
                else {
                    z = static_cast<double>(-signbit(Op.dcost));//[TODO:doublecheck]
                    TRACE("non-positive expected reduction: should not occur \n");//warning
                }
                if(z > Op.zMin){ // Op.zMin=0
                    fwdPassDone = 1;
                    break;
                }
            }
            if(!fwdPassDone) alpha = sqrt(-1.0);
            gettimeofday(&tend_time_fwd,NULL);
            Op.time_forward(iter) = (static_cast<double>(1000*(tend_time_fwd.tv_sec-tbegin_time_fwd.tv_sec)+((tend_time_fwd.tv_usec-tbegin_time_fwd.tv_usec)/1000)))/1000.0;
        }
        
        //====== STEP 4: accept step (or not), draw graphics, print status
        if (Op.debug_level > 1 && Op.last_head == Op.print_head){
            Op.last_head = 0;
            TRACE("iteration,\t cost, \t reduction, \t expected, \t gradient, \t log10(lambda) \n");
        }
        
        if(fwdPassDone){
            // print status
            if (Op.debug_level > 1){
                if(!debugging_print) printf("%-14d%-12.6g%-15.3g%-15.3g%-19.3g%-17.1f\n", iter+1, accumulate(costList.begin(), costList.end(), 0.0), Op.dcost, Op.expected, Op.g_norm, log10(Op.lambda));
                Op.last_head = Op.last_head+1;
            }

            Op.dlambda = min(Op.dlambda / Op.lambdaFactor, 1.0/Op.lambdaFactor);
            Op.lambda = Op.lambda * Op.dlambda * (Op.lambda > Op.lambdaMin);

            // accept changes
            xList = updatedxList;
            uList = updateduList;
            costList = costListNew;
            newDeriv = 1;

            // terminate ?
            // TODO: add constraint tolerance check
            if(Op.dcost < Op.tolFun) {
                if(Op.debug_level >= 1)
                    TRACE(("\nSUCCESS: cost change < tolFun\n"));
            
                break;
            }
        }
        else { // no cost improvement
            // increase lambda
            Op.dlambda= max(Op.dlambda * Op.lambdaFactor, Op.lambdaFactor);
            Op.lambda= max(Op.lambda * Op.dlambda, Op.lambdaMin);

            // if(o->w_pen_fact2>1.0) {
            //     o->w_pen_l= min(o->w_pen_max_l, o->w_pen_l*o->w_pen_fact2);
            //     o->w_pen_f= min(o->w_pen_max_f, o->w_pen_f*o->w_pen_fact2);
            //     forward_pass(o->nominal, o, 0.0, &o->cost, 1);
            // }
            
            // print status
            if(Op.debug_level >= 1){
                if(!debugging_print) printf("%-14d%-12.9s%-15.3g%-15.3g%-19.3g%-17.1f\n", iter+1, "No STEP", Op.dcost, Op.expected, Op.g_norm, log10(Op.lambda));
                Op.last_head = Op.last_head+1;
            }

            // terminate ?
            if(Op.lambda > Op.lambdaMax) {
                if(Op.debug_level >= 1)
                    TRACE(("\nEXIT: lambda > lambdaMax\n"));
                break;
            }
        }
    }

    Op.iterations = iter;

    if(!backPassDone) {
        if(Op.debug_level >= 1)
            TRACE(("\nEXIT: no descent direction found.\n"));
        
        return;    
    } else if(iter >= Op.max_iter) {
        if(Op.debug_level >= 1)
            TRACE(("\nEXIT: Maximum iterations reached.\n"));
        
        return;
    }
}

void ILQRSolver_TRK::initializeTraj()
{
    xList[0] = Op.xInit;
    // commandVec_t initCommand;
    // initCommand.setZero();
    // (low priority) TODO: implement control limit selection
    // (low priority) TODO: initialize trace data structure

    initFwdPassDone = 0;
    diverge = 1;

    // cout << "Op alphaList size: " << Op.alphaList.size() << endl; // 11
    
    for(int alpha_index = 0; alpha_index < Op.alphaList.size(); alpha_index++){
        alpha = Op.alphaList[alpha_index]; // alpha = 1, 

        // cout << "alpha: " << alpha << endl;

        for(unsigned int i=0;i<N;i++)
        {
            // uList[i] = zeroCommand;
//             zeroCommand << 0,0,0,0,0.1,0.1,0;
//            zeroCommand << 0,0,0,0,0,0,0;
            // initCommand << 0,0,0.2,0,0.2,0,0;
            uList[i] = initCommand[i];
        }

        doForwardPass();
        //simplistic divergence test
        int diverge_element_flag = 0;
        for(unsigned int i = 0; i < xList.size(); i++){
            for(unsigned int j = 0; j < xList[i].size(); j++){
                if(fabs(xList[i](j,0)) > 1e8) // checking the absolute value
                    diverge_element_flag = 1;
            }
        }
        if(!diverge_element_flag){
            cout << "Not Diverge?" << endl;
            diverge = 0;
            break;
        }
    }
    
    initFwdPassDone = 1;
    xList = updatedxList;
    
    //constants, timers, counters
    newDeriv = 1; //flgChange
    Op.lambda= Op.lambdaInit;
    Op.w_pen_l= Op.w_pen_init_l;
    Op.w_pen_f= Op.w_pen_init_f;
    Op.dcost = 0;
    Op.expected = 0;
    Op.print_head = 6;
    Op.last_head = Op.print_head;
}

void ILQRSolver_TRK::standardizeParameters(tOptSet *o) {
    o->n_alpha = 11;
    o->tolFun = 1e-4;
    o->tolConstraint = 1e-7; // TODO: to be modified
    o->tolGrad = 1e-4;
    o->max_iter = 500;
    o->lambdaInit = 1;
    o->dlambdaInit = 1;
    o->lambdaFactor = 1.6;
    o->lambdaMax = 1e10;
    o->lambdaMin = 1e-6;
    o->regType = 1;
    o->zMin = 0.0;
    o->debug_level = 2; // == verbosity in matlab code
    o->w_pen_init_l = 1.0; // TODO: to be modified
    o->w_pen_init_f = 1.0; // TODO: to be modified
    o->w_pen_max_l = 1e100;//set to INF originally
    o->w_pen_max_f = 1e100;//set to INF originally
    o->w_pen_fact1 = 4.0; // 4...10 Bertsekas p. 123
    o->w_pen_fact2 = 1.0; 
    o->print = 2;
}

void ILQRSolver_TRK::doBackwardPass()
{    
    if(Op.regType == 1)
        lambdaEye = Op.lambda*stateMat_t::Identity();
    else
        lambdaEye = Op.lambda*stateMat_t::Zero();

    diverge = 0;
    
    g_norm_sum = 0.0;
    Vx[N] = costFunction->getcx()[N];
    Vxx[N] = costFunction->getcxx()[N];
    dV.setZero();

    for(int i=static_cast<int>(N-1);i>=0;i--)
    {
        Qx = costFunction->getcx()[i] + dynamicModel->getfxList()[i].transpose()*Vx[i+1];
        Qu = costFunction->getcu()[i] + dynamicModel->getfuList()[i].transpose()*Vx[i+1];
        Qxx = costFunction->getcxx()[i] + dynamicModel->getfxList()[i].transpose()*(Vxx[i+1])*dynamicModel->getfxList()[i];
        Quu = costFunction->getcuu()[i] + dynamicModel->getfuList()[i].transpose()*(Vxx[i+1])*dynamicModel->getfuList()[i];
        Qux = costFunction->getcux()[i] + dynamicModel->getfuList()[i].transpose()*(Vxx[i+1])*dynamicModel->getfxList()[i];

        if(Op.regType == 1)
            QuuF = Quu + Op.lambda*commandMat_t::Identity();
        else
            QuuF = Quu;
        
        // if(enableFullDDP)
        // {
            //Qxx += dynamicModel->computeTensorContxx(nextVx);
            //Qux += dynamicModel->computeTensorContux(nextVx);
            //Quu += dynamicModel->computeTensorContuu(nextVx);

            // Qxx += dynamicModel->computeTensorContxx(Vx[i+1]);
            // Qux += dynamicModel->computeTensorContux(Vx[i+1]);
            // Quu += dynamicModel->computeTensorContuu(Vx[i+1]);
            // QuuF += dynamicModel->computeTensorContuu(Vx[i+1]);
        // }
        
        QuuInv = QuuF.inverse();

        if(!isPositiveDefinite(Quu))
        {
            
            //To be Implemented : Regularization (is Quu definite positive ?)
            TRACE("Quu is not positive definite");
            if(Op.lambda==0.0) Op.lambda += 1e-4;
            else Op.lambda *= 10;
            backPassDone = 0;
            break;
        }

        // if(enableQPBox)
        // {
        //     //TRACE("Use Box QP");
        //     nWSR = 10; //[to be checked]
        //     H = Quu;
        //     g = Qu;
        //     lb = lowerCommandBounds - uList[i];
        //     ub = upperCommandBounds - uList[i];
        //     qp->init(H.data(),g.data(),lb.data(),ub.data(),nWSR);
        //     qp->getPrimalSolution(xOpt);
        //     k = Map<commandVec_t>(xOpt);
        //     K = -QuuInv*Qux;
        //     for(unsigned int i_cmd=0;i_cmd<commandNb;i_cmd++)
        //     {
        //         if((k[i_cmd] == lowerCommandBounds[i_cmd]) | (k[i_cmd] == upperCommandBounds[i_cmd]))
        //         {
        //             K.row(i_cmd).setZero();
        //         }
        //     }
        // }
        if(!enableQPBox)
        {
            // Cholesky decomposition by using upper triangular matrix
            //TRACE("Use Cholesky decomposition");
            Eigen::LLT<MatrixXd> lltOfQuuF(QuuF);
            Eigen::MatrixXd L = lltOfQuuF.matrixU(); 
            //assume QuuF is positive definite
            
            //A temporary solution: check the non-PD case
            if(lltOfQuuF.info() == Eigen::NumericalIssue)
                {
                    diverge = i;
                    TRACE("Possibly non semi-positive definitie matrix!");
                    return;
                }

            Eigen::MatrixXd L_inverse = L.inverse();
            k = - L_inverse*L.transpose().inverse()*Qu;
            K = - L_inverse*L.transpose().inverse()*Qux;
        }

        //update cost-to-go approximation
        dV(0) += k.transpose()*Qu;
        scalar_t c_mat_to_scalar;
        c_mat_to_scalar = 0.5*k.transpose()*Quu*k;
        dV(1) += c_mat_to_scalar(0,0);
        Vx[i] = Qx + K.transpose()*Quu*k + K.transpose()*Qu + Qux.transpose()*k;
        Vxx[i] = Qxx + K.transpose()*Quu*K+ K.transpose()*Qux + Qux.transpose()*K;
        Vxx[i] = 0.5*(Vxx[i] + Vxx[i].transpose());

        kList[i] = k;
        KList[i] = K;

        g_norm_max= 0.0;
        for(unsigned int j=0; j<commandSize; j++) {
            g_norm_i = fabs(kList[i](j,0)) / (fabs(uList[i](j,0))+1.0);
            if(g_norm_i > g_norm_max) g_norm_max = g_norm_i;
        }
        g_norm_sum += g_norm_max;
    }
    Op.g_norm = g_norm_sum/(static_cast<double>(Op.n_hor));
}

void ILQRSolver_TRK::doForwardPass()
{
    updatedxList[0] = Op.xInit;
    int nargout = 2;

    stateVec_t x_unused;
    x_unused.setZero();
    commandVec_t u_NAN_loc;
    u_NAN_loc << sqrt(-1.0); // sqrt(-1)=NaN. After this line, u_nan has nan for [0] and garbage for rest
    isUNan = 0;

    //[TODO: to be optimized]
    if(!initFwdPassDone){
        //TRACE("initial forward pass");
        for(unsigned int i=0;i<N;i++) {
            updateduList[i] = uList[i];
            // running cost, state, input
            dynamicModel->kuka_arm_dyn_cst_min_output(nargout, updatedxList[i], updateduList[i], xList_bar[i], uList_bar[i], isUNan, updatedxList[i+1], costFunction);
            costList[i] = costFunction->getc();
        }
        // getting final cost, state, input=NaN
        isUNan = 1;
        dynamicModel->kuka_arm_dyn_cst_min_output(nargout, updatedxList[N], u_NAN_loc, xList_bar[N], u_NAN_loc, isUNan, x_unused, costFunction);
        costList[N] = costFunction->getc();
    }
    else {
        for(unsigned int i=0;i<N;i++) {
            updateduList[i] = uList[i] + alpha*kList[i] + KList[i]*(updatedxList[i]-xList[i]);
            dynamicModel->kuka_arm_dyn_cst_min_output(nargout, updatedxList[i], updateduList[i], xList_bar[i], uList_bar[i], isUNan, updatedxList[i+1], costFunction);
            costListNew[i] = costFunction->getc();
        }
        isUNan = 1;
        dynamicModel->kuka_arm_dyn_cst_min_output(nargout, updatedxList[N], u_NAN_loc, xList_bar[N], u_NAN_loc, isUNan, x_unused, costFunction);
        costListNew[N] = costFunction->getc();
    }
}

ILQRSolver_TRK::traj ILQRSolver_TRK::getLastSolvedTrajectory()
{
    lastTraj.xList = xList;
    // for(unsigned int i=0;i<N+1;i++)lastTraj.xList[i] += xgoal;//retrieve original state with xgoal
    lastTraj.uList = uList;
    lastTraj.iter = iter;
    lastTraj.finalCost = accumulate(costList.begin(), costList.end(), 0.0);
    lastTraj.finalGrad = Op.g_norm;
    lastTraj.finalLambda = log10(Op.lambda);
    lastTraj.time_forward = Op.time_forward;
    lastTraj.time_backward = Op.time_backward;
    lastTraj.time_derivative = Op.time_derivative;
    return lastTraj;
}

bool ILQRSolver_TRK::isPositiveDefinite(const commandMat_t & Quu_p)
{
    //Eigen::JacobiSVD<commandMat_t> svd_Quu (Quu, ComputeThinU | ComputeThinV);
    Eigen::VectorXcd singular_values = Quu_p.eigenvalues();

    for(long i = 0; i < Quu_p.cols(); ++i)
    {
        if (singular_values[i].real() < 0.)
        {
            TRACE("Matrix is not SDP");
            return false;
        }
    }
    return true;
}

}  // namespace kuka_iiwa_arm
}  // namespace traj_gen
}  // namespace drake
