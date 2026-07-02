/*
本文件实现了基于 Bezier 曲线的轨迹预测算法。主要思路是：

构造代价矩阵 Q 和 M
对于规划问题，利用 Bezier 曲线描述一段轨迹，其系数为多项式系数。函数 getQ 用于计算某一段轨迹的成本矩阵 Q_k（反映轨迹平滑性等指标），而 getM 用于构造归一化矩阵 M_k，用于将原始多项式系数归一化。

历史预测数据处理
从传入的完整预测列表（predict_list_complete，每个元素是 4 维向量，其中前三个分量代表位置，第四个分量代表时间）中提取出预测的轨迹点（predict_list）以及对应的时间信息（predict_list_time），并利用时间差构造历史权重（history_weight_list），这里采用 tanh 函数给后期数据赋予不同权重。

QP 优化问题构造
结合轨迹段时间信息、Bezier 基函数以及历史权重，构造出一个二次规划问题。其目标函数包含两个部分：

与预测轨迹的距离误差项（distance_Q 部分）
控制输入的平滑性或加速度惩罚（Lambda_ACC 权重下的 Q 部分）
最终构造出 Hessian 矩阵 M_QM 和线性项 c，然后利用 qpOASES（或类似求解器）求解二次规划，得到最优的多项式系数。

结果提取
求解成功后，将解向量拆分重构为每段轨迹的系数矩阵 PolyCoeff 以及时间信息 PolyTime，同时计算目标函数值 obj。
整个流程实现了从预测数据到优化生成平滑轨迹的转换，为后续轨迹跟踪提供了预规划参考。
*/
#include <bezier_prediction/bezier_predict.h>
#include <math.h>
#include <ros/console.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
using namespace std;
using namespace Eigen;

// 计算第 seg_index 段的 Q 矩阵（用于代价计算）
// 输入参数：vars_number 为多项式系数数量（traj_order+1），Time 为各段时间数组，seg_index 指定当前段
Eigen::MatrixXd Bezierpredict::getQ(const int vars_number, const vector<double> Time, const int seg_index) {
    // 初始化 Q_k 为零矩阵
    Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(vars_number, vars_number);
    // d_order 为求导阶数，通常为 (traj_order+1)/2 - 1，对应二次连续性等约束
    int d_order = (traj_order + 1) / 2 - 1;  // for us, is 3 here
    for (int i = 0; i < vars_number; i++) {
        for (int j = 0; j < vars_number; j++) {
            // 只有当 i 和 j 均大于等于 d_order 时才有意义
            if (i - d_order >= 0 && j - d_order >= 0) {
                // 计算 Q_k(i,j) 的值：主要形式为 (factorial(i)/factorial(i-d_order))*(factorial(j)/factorial(j-d_order))
                // 除以 (i+j-2*d_order+1) 后乘以当前段时间的相应幂次
                Q_k(i, j) = (factorial(i) / factorial(i - d_order)) * ((factorial(j) / factorial(j - d_order))) /
                            (i + j - 2 * d_order + 1) * pow(Time[seg_index], (i + j - 2 * d_order + 1));  // Q of one segment
            }
        }
    }
    // 返回当前段 Q 矩阵
    return Q_k;
}
// 计算第 seg_index 段的 M 矩阵
// M 矩阵用于对多项式系数归一化，依赖于时间因子
Eigen::MatrixXd Bezierpredict::getM(const int vars_number, const vector<double> Time, const int seg_index) {
    MatrixXd M_k = MatrixXd::Zero(vars_number, vars_number);
    // 构造 t_pow 向量，每个元素为 (Time[seg_index])^i
    VectorXd t_pow = VectorXd::Zero(vars_number);
    for (int i = 0; i < vars_number; i++) {
        t_pow(i) = pow(Time[seg_index], i);
    }
    // 先复制预定义的 M 矩阵（该矩阵通常在类中提前定义好）
    M_k = M;
    // 对每一行除以相应的 t_pow(i)，实现归一化
    for (int i = 0; i < vars_number; i++) {
        M_k.row(i) = M_k.row(i) / t_pow(i);
    }
    return M_k;
}

/*
函数功能：
该函数用来根据输入的目标预测点（包含位置和时间信息）生成Bezier轨迹预测，并通过构造与求解二次规划（QP）问题，得到平滑多项式轨迹系数。函数返回QP求解的错误码（ierr），等于0表示求解成功。

输入参数：
max_vel：目标最大允许速度，用于构造速度约束。
max_acc：目标最大允许加速度，用于构造加速度约束。
predict_list_complete：包含多个预测点数据的向量，每个元素为 Eigen::Vector4d，其中前三个值表示目标三维位置（X, Y, Z），第四个值表示对应时间戳或相对时间。

全局变量/成员变量：
segs：当前采用的分段数（本例中固定为1段）。
_MAX_SEG：预定义的预测点总数，表示对历史数据使用多少点作为参考。
traj_order：多项式轨迹的阶数，变量数量为 traj_order+1。
C_：预先定义的常数数组，用来加权计算多项式约束（具体含义与设计有关）。
_TIME_INTERVAL, _PREDICT_SEG：与预测时间延伸有关的参数。
其它如 _Q, _M, PolyCoeff, PolyTime, Lambda_ACC 等变量用于构造和存储QP问题的数据及优化结果。
*/
int Bezierpredict::TrackingGeneration(
    const double max_vel,
    const double max_acc,
    vector<Eigen::Vector4d> predict_list_complete) {
    // 记录开始时间，用于统计求解时间
    ros::Time time_1 = ros::Time::now();
    // 初始化分段数量（这里仅使用 1 段）
    segs = 1;
    vector<double> time_intervals;        // 每段持续时间（这里只有一段）
    vector<double> total_time_intervals;  // 每段总时间（包含预测时域延伸）
    vector<Vector3d> predict_list;        // 存储预测出的 3D 位置（x,y,z）
    vector<double> predict_list_time;     // 存储每个预测点对应的时间（相对于初始时间）

    vector<double> history_weight_list;  // 历史权重数组，用于调整不同预测点对优化的影响
    history_time_total = 0;              // 初始化所有预测时间累积值
    int init_flag = 0;                   // 用于记录是否设置初始时间
    /*
    详细说明：
    循环遍历 _MAX_SEG 个预测点，提取位置数据并保存到 predict_list 中；
    对于第一次循环，将预测点时间作为初始时间 history_time_init，后续每个预测点相对于这个初始时间计算差值，存入 predict_list_time。
    同时用 history_time_total 保存最新预测点相对于起始时刻的时间差。
    */
    // 遍历 _MAX_SEG 个预测点
    for (int i = 0; i < _MAX_SEG; i++) {
        // 取前 3 个元素为位置
        predict_list.push_back(predict_list_complete[i].head(3));
        if (!init_flag) {
            // 使用第一个预测点的时间作为初始时间 history_time_init
            history_time_init = predict_list_complete[i][3];
            init_flag = 1;
        }
        // 计算当前预测点相对于初始时刻的时间差，累积到 history_time_total
        history_time_total = predict_list_complete[i][3] - history_time_init;
        predict_list_time.push_back(history_time_total);
        // ROS_INFO_STREAM("pre_time: " << predict_list_time[i]);
    }
    // ROS_INFO_STREAM("tanh weight: " );
    // 计算每个预测点的权重：根据与最后一个预测点时间差的倒数，经过 tanh 函数映射
    for (int i = 0; i < _MAX_SEG; i++) {
        double tanh_input = predict_list_time[_MAX_SEG - 1] - predict_list_time[i];
        if (!tanh_input) {  // 当时间差为0时，直接赋值权重1
            history_weight_list.push_back(1);
        } else {
            tanh_input = 1.0 / tanh_input;
            // 1.2为缩放因子，实际使用 tanh 将输入映射到(-1,1)区间
            history_weight_list.push_back(tanh(1.2 * tanh_input));
            // ROS_INFO_STREAM("round: " <<i << " " << tanh(6 * tanh_input));
        }
    }

    // 对于当前段，设置时间间隔。这里直接用 history_time_total 作为基础
    for (int i = 0; i < segs; i++) {
        // time_intervals.push_back(time(i));
        // 使用 history_time_total 作为当前段的基本持续时间
        time_intervals.push_back(history_time_total);
        // 总时间为基础时间加上额外延伸预测的时间：_TIME_INTERVAL * _PREDICT_SEG
        total_time_intervals.push_back(time_intervals[i] + (_TIME_INTERVAL * _PREDICT_SEG));
    }

    // 取最后一个预测点作为目标终点
    Vector3d end_p = predict_list[_MAX_SEG - 1];
    int constrain_flag = 0;
    // 这里可以设置额外的约束条件，比如时间差超过某值则激活约束
    // double time_diff = (ros::Time::now().toSec() - history_time_init)- predict_list_time[_MAX_SEG-1];
    // //ROS_INFO_STREAM("time diff: " << time_diff);
    // if(time_diff >= 0.25){
    //     constrain_flag = 1;
    // }

    // 多项式变量设置与QP问题变量初始化
    // 多项式变量数量，每个维度轨迹多项式有 traj_order+1 个系数，例如6阶多项式则变量数为6
    int vars_number = traj_order + 1;       // 例如 6
    int all_vars_number = 3 * vars_number;  // XYZ 分量总数，例如 18
    int nx = segs * 3 * vars_number;        // 优化变量总数
    // 定义存放QP问题中线性项 c 与各变量边界（上界 xupp 和下界 xlow），以及它们的标记变量
    double c[nx];
    double xupp[nx];
    char ixupp[nx];
    double xlow[nx];
    char ixlow[nx];
    for (int i = 0; i < nx; i++) {
        c[i] = 0.0;
        xlow[i] = 0.0;
        ixlow[i] = 0;
        xupp[i] = 0.0;
        ixupp[i] = 0;
    }
    // 定义等式约束右侧 b（这里对应轨迹终点的 x, y, z 坐标），用于设置边界条件。
    int my = 3;
    double b[3];
    b[0] = end_p[0];  // end_p
    b[1] = end_p[1];
    b[2] = end_p[2];

    /*
    构造 A 矩阵（QP约束矩阵）的稀疏表示       目的是将时间信息引入优化问题中，一般用作边界条件或端点条件的线性约束。
    使用基函数（可能为贝塞尔或 Bernstein 多项式基函数）计算各个系数在当前时刻下的贡献。
    */
    int nnzA = vars_number * 3;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    int nn_idx = 0;
    int row_idx = 0;

    double curr_t = time_intervals[0];
    double total_t = total_time_intervals[0];
    // 对于每个坐标轴（x,y,z），构造与多项式系数相关的约束（例如起始点边界条件）
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < vars_number; j++) {
            // 计算多项式基函数值：使用 C_[j]（预定义常数）乘以时间比率函数
            dA[nn_idx] = C_[j] * pow(curr_t / total_t, j) * pow((1 - curr_t / total_t), (traj_order - j));
            // 设置约束矩阵的行、列索引
            irowA[nn_idx] = row_idx;
            jcolA[nn_idx] = i * vars_number + j;
            nn_idx++;
        }
        row_idx++;  // 每个坐标一行
    }
    /*
    分两部分设置限制：
    速度约束（对多项式导数的一次微分）
    加速度约束（对多项式的二次微分）
    每个约束均用上下界数组 clow 与 cupp 来表示，iclow、icupp 的标记表示该约束是否存在（通常为1表示有效）。
    */
    // 高阶约束数量包含速度和加速度约束
    int high_order_con_num = 3 * (vars_number - 1) * segs + 3 * (vars_number - 2) * segs;
    const int mz = high_order_con_num;
    char iclow[mz];
    char icupp[mz];
    double clow[mz];
    double cupp[mz];

    int m_idx = 0;
    // 设置速度约束：对于每个坐标和每段的每个速度项，其上下界设为 -max_vel 和 max_vel
    for (int i = 0; i < 3 * (vars_number - 1) * segs; i++) {
        iclow[m_idx] = 1;
        icupp[m_idx] = 1;
        clow[m_idx] = -max_vel;
        cupp[m_idx] = max_vel;
        m_idx++;
    }
    // 设置加速度约束：对于每个坐标和每段的每个加速度项，其上下界设为 -max_acc 和 max_acc
    for (int i = 0; i < 3 * (vars_number - 2) * segs; i++) {
        iclow[m_idx] = 1;
        icupp[m_idx] = 1;
        clow[m_idx] = -max_acc;
        cupp[m_idx] = max_acc;
        m_idx++;
    }

    // 构造稀疏矩阵 C 用于表示速度和加速度约束
    /*
    速度约束利用差分原理构造相邻多项式系数之间的约束，用来近似一阶导数（速度）；
    加速度约束利用连续的三项构造二阶导数限制，确保生成轨迹中加速度不超过最大值。
    采用稀疏矩阵的形式存储索引和值，便于后续构造QP问题。
    */
    int nnzC = segs * (vars_number - 1) * 2 * 3 + segs * (vars_number - 2) * 3 * 3;
    int irowC[nnzC];
    int jcolC[nnzC];
    double dC[nnzC];
    nn_idx = 0;
    row_idx = 0;
    // 速度约束部分：对每个段、每个坐标，对每个时间间隔构造一对约束（负与正）
    for (int k = 0; k < segs; k++) {
        for (int i = 0; i < 3; i++) {  // for x, y, z loop
            for (int j = 0; j < traj_order; j++) {
                dC[nn_idx] = -1.0 * traj_order / total_time_intervals[k];
                dC[nn_idx + 1] = 1.0 * traj_order / total_time_intervals[k];
                irowC[nn_idx] = row_idx;
                irowC[nn_idx + 1] = row_idx;
                jcolC[nn_idx] = k * all_vars_number + i * vars_number + j;
                jcolC[nn_idx + 1] = k * all_vars_number + i * vars_number + j + 1;
                row_idx++;
                nn_idx += 2;
            }
        }
    }
    // 加速度约束部分：类似地对二阶导数建立3项连续约束
    for (int k = 0; k < segs; k++) {
        double scale_k = pow(total_time_intervals[k], 2);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < traj_order - 1; j++) {
                dC[nn_idx] = 1.0 * traj_order * (traj_order - 1) / scale_k;
                dC[nn_idx + 1] = -2.0 * traj_order * (traj_order - 1) / scale_k;
                dC[nn_idx + 2] = 1.0 * traj_order * (traj_order - 1) / scale_k;

                irowC[nn_idx] = row_idx;
                irowC[nn_idx + 1] = row_idx;
                irowC[nn_idx + 2] = row_idx;

                jcolC[nn_idx] = k * all_vars_number + i * vars_number + j;
                jcolC[nn_idx + 1] = k * all_vars_number + i * vars_number + j + 1;
                jcolC[nn_idx + 2] = k * all_vars_number + i * vars_number + j + 2;

                row_idx++;
                nn_idx += 3;
            }
        }
    }
    /*
    _Q 表示目标函数中的权重矩阵，一般与轨迹平滑度、能量消耗等有关；
    _M 是多项式系数变换矩阵，可以将原始优化变量投影到轨迹参数空间；
    这两个矩阵的构造依赖于函数 getQ() 与 getM()，设计时需根据具体优化问题确定。
    */
    // 初始化 Q 和 M 矩阵，其尺寸为：每段3个坐标，每个坐标有vars_number个变量
    _Q = MatrixXd::Zero(vars_number * segs * 3, vars_number * segs * 3);
    _M = MatrixXd::Zero(vars_number * segs * 3, vars_number * segs * 3);
    for (int i = 0; i < segs; i++) {
        for (int j = 0; j < 3; j++) {
            // getQ: 根据变量数、时间间隔等参数构造该坐标的 Hessian 块矩阵 Q
            _Q.block(i * all_vars_number + j * vars_number, i * all_vars_number + j * vars_number, vars_number, vars_number) = getQ(vars_number, total_time_intervals, i);
            // getM: 构造相应的变换矩阵，用于将优化变量映射为实际轨迹参数
            _M.block(i * all_vars_number + j * vars_number, i * all_vars_number + j * vars_number, vars_number, vars_number) = getM(vars_number, total_time_intervals, i);
        }
    }
    /*
    该部分将预测数据的时间与位置信息通过函数 getCt()（用户自定义）转换成一个线性表达，再加权累加；
    同时构造了 distance_Q 矩阵，用于描述与目标预测点距离惩罚（鼓励轨迹接近预测点）；
    最后用 _M 对 Ct 进行变换，生成最终QP问题的线性项向量 c。
    */
    // 构造 Ct，用于线性项的生成
    MatrixXd Ct = MatrixXd::Zero(1, segs * 3 * (traj_order + 1));
    MatrixXd distance_Q = MatrixXd::Zero(vars_number * segs * 3, vars_number * segs * 3);
    double t_base = 0;
    for (int i = 0; i < segs; i++) {
        if (i >= 1)
            t_base += time_intervals[i - 1];
        for (double j = 0; j < _MAX_SEG; j += 1) {
            Ct.block(0, i * 3 * (traj_order + 1), 1, 3 * (traj_order + 1)) += getCt(predict_list_time[j], predict_list[j]) * history_weight_list[j];
            distance_Q.block(all_vars_number * i, all_vars_number * i, all_vars_number, all_vars_number) += getdistance_Q(predict_list_time[j]) * history_weight_list[j];
        }
    }
    // ROS_INFO_STREAM("asdf" << Ct);
    Ct = Ct * _M;
    /*
     */
    // MatrixXd snapQ = _Q;
    // double sim_weight = 0.0;
    // 将 Ct 的内容拷贝到数组 c（作为QP问题的线性项）
    for (int i = 0; i < nx; i++)
        c[i] = Ct(0, i);

    _Q = 2 * (Lambda_ACC * _MAX_SEG * _Q + distance_Q);
    // ROS_INFO_STREAM("asdf" << _Q);
    // 构造稀疏表示的 Hessian 部分
    /*
    M_QM 是将 _Q 经过 M 变换后的 Hessian 矩阵，用于后续QP求解；
    接下来构造稀疏矩阵表示的 Hessian，利用上三角部分（因为 Hessian 对称）。
    */
    MatrixXd M_QM;
    M_QM = MatrixXd::Zero(_M.rows(), _M.cols());
    // ROS_INFO_STREAM  ("M:rows"<<_M.rows()<<"col"<<_M.cols()<<"Q:rows"<<_Q.rows()<<"col:"<<_Q.cols());
    M_QM = _M.transpose() * _Q * _M;

    const int nnzQ = 3 * segs * (traj_order + 1) * (traj_order + 2) / 2;  // n(n+1)/2
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];

    int sub_shift = 0;
    int Q_idx = 0;

    for (int k = 0; k < segs; k++) {
        for (int p = 0; p < 3; p++)
            for (int i = 0; i < vars_number; i++)
                for (int j = 0; j < vars_number; j++)
                    if (i >= j) {
                        irowQ[Q_idx] = sub_shift + p * vars_number + i;
                        jcolQ[Q_idx] = sub_shift + p * vars_number + j;
                        dQ[Q_idx] = M_QM(sub_shift + p * vars_number + i, sub_shift + p * vars_number + j);
                        // dQ[Q_idx] = 1;
                        Q_idx++;
                    }
        sub_shift += all_vars_number;
    }
    // my=0;
    // nnzA=0;
    // 此处构造QP求解器（略去约束部分代码，可根据是否有额外约束选择）
    QpGenSparseMa27* qp;
    QpGenData* prob;
    QpGenVars* vars;
    QpGenResiduals* resid;
    GondzioSolver* s;
    if (constrain_flag) {
        int my = 0;
        double* b = 0;
        int nnzA = 0;
        int* irowA = 0;
        int* jcolA = 0;
        double* dA = 0;

        int nn_idx = 0;
        int row_idx = 0;
        qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
        // cout<<"irowQ: "<<irowQ[nnzQ-1]<<"jcolQ: "<<jcolQ[nnzQ-1];
        // cout<<"nx: "<<nx<<" my: "<<my<<" mz: "<<mz<<" nnzQ: "<<nnzQ<<" nnzA: "<<nnzA<<" nnzC: "<<nnzC<<" size: "<<M_QM.cols()<<" "<<M_QM.rows();
        prob = (QpGenData*)qp->copyDataFromSparseTriple(
            c, irowQ, nnzQ, jcolQ, dQ,
            xlow, ixlow, xupp, ixupp,
            irowA, nnzA, jcolA, dA, b,
            irowC, nnzC, jcolC, dC,
            clow, iclow, cupp, icupp);

        vars = (QpGenVars*)qp->makeVariables(prob);
        resid = (QpGenResiduals*)qp->makeResiduals(prob);
        s = new GondzioSolver(qp, prob);
    } else {
        qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
        // cout<<"irowQ: "<<irowQ[nnzQ-1]<<"jcolQ: "<<jcolQ[nnzQ-1];
        // cout<<"nx: "<<nx<<" my: "<<my<<" mz: "<<mz<<" nnzQ: "<<nnzQ<<" nnzA: "<<nnzA<<" nnzC: "<<nnzC<<" size: "<<M_QM.cols()<<" "<<M_QM.rows();
        prob = (QpGenData*)qp->copyDataFromSparseTriple(
            c, irowQ, nnzQ, jcolQ, dQ,
            xlow, ixlow, xupp, ixupp,
            irowA, nnzA, jcolA, dA, b,
            irowC, nnzC, jcolC, dC,
            clow, iclow, cupp, icupp);

        vars = (QpGenVars*)qp->makeVariables(prob);
        resid = (QpGenResiduals*)qp->makeResiduals(prob);
        s = new GondzioSolver(qp, prob);
    }

    // Turn Off/On the print of the solving process
    // s->monitorSelf();
    int ierr = s->solve(prob, vars, resid);
    if (ierr == 0) {
        double d_var[nx];
        vars->x->copyIntoArray(d_var);
        // cout<<"d_var="<<d_var;
        // int temp_count=0;
        // for(int kk=0;kk<nx;kk++)
        // {
        //     cout<<"d_var="<<d_var[kk];
        //     temp_count++;
        //     if(temp_count%10==0)
        //         cout<<"    count="<<temp_count<<endl;
        // }

        // cout.precision(4);
        // cout << "Solution: \n";
        // vars->x->writefToStream(cout, "x[%{index}] = %{value}");
        vars->x->copyIntoArray(d_var);

        PolyCoeff = MatrixXd::Zero(segs, all_vars_number);
        PolyTime = VectorXd::Zero(segs);
        obj = 0.0;

        int var_shift = 0;

        MatrixXd Q_o(vars_number, vars_number);
        //    int s1d1CtrlP_num = traj_order + 1;
        //    int s1CtrlP_num   = 3 * s1d1CtrlP_num;
        // int min_order_l = floor(minimize_order);
        // int min_order_u = ceil (minimize_order);

        for (int i = 0; i < segs; i++) {
            PolyTime(i) = total_time_intervals[i];

            for (int j = 0; j < all_vars_number; j++) {
                PolyCoeff(i, j) = d_var[j + var_shift];
                // cout<<"coeff in is  "<<PolyCoeff(i , j)<<"i="<<i<<"  j="<<j<<endl;
            }
            var_shift += all_vars_number;
        }
        MatrixXd flat_poly = MatrixXd::Zero(1, segs * all_vars_number);
        for (int i = 0; i < segs; i++) {
            flat_poly.block(0, i * all_vars_number, 1, all_vars_number) = PolyCoeff.block(i, 0, 1, all_vars_number);
        }
        /*double sum_time;
        for(int i=0;i<time_intervals.size();i++)
            sum_time+=time_intervals[i];
        ROS_INFO_STREAM("CT: "<<Ct*flat_poly.transpose());
        ROS_INFO_STREAM("distance: "<<flat_poly* distance_Q*flat_poly.transpose());
        ROS_INFO_STREAM("snap Q: "<<flat_poly*snapQ*flat_poly.transpose());
        ROS_INFO_STREAM("time : "<<sum_time);*/

    } else if (ierr == 3)
        ROS_ERROR("Front Bezier Predict: The program is provably infeasible, check the formulation");
    else if (ierr == 4)
        ROS_ERROR("Front Bezier Predict: The program is very slow in convergence, may have numerical issue");
    else
        ROS_ERROR("Front Bezier Predict: Solver numerical error");

    ros::Time time_2 = ros::Time::now();
    // ROS_INFO_STREAM("Bezier time consumed:" << (time_2 - time_1).toSec()*1000<<" ms");
    return ierr;
}
