#include <bluerov2_dobmpc/bluerov2_dob.h>

// Initialize MPC
BLUEROV2_DOB::BLUEROV2_DOB(ros::NodeHandle& nh)
{
    // read parameter
    nh.getParam("/bluerov2_dob_node/auto_yaw",AUTO_YAW);
    nh.getParam("/bluerov2_dob_node/read_wrench",READ_WRENCH);
    nh.getParam("/bluerov2_dob_node/compensate_d",COMPENSATE_D);
    nh.getParam("/bluerov2_dob_node/ref_traj", REF_TRAJ);
    nh.getParam("/bluerov2_dob_node/applied_forcex", WRENCH_FX);
    nh.getParam("/bluerov2_dob_node/applied_forcey", WRENCH_FY);
    nh.getParam("/bluerov2_dob_node/applied_forcez", WRENCH_FZ);
    nh.getParam("/bluerov2_dob_node/applied_torquez", WRENCH_TZ);
    nh.getParam("/bluerov2_dob_node/disturbance_x", solver_param.disturbance_x);
    nh.getParam("/bluerov2_dob_node/disturbance_y", solver_param.disturbance_y);
    nh.getParam("/bluerov2_dob_node/disturbance_z", solver_param.disturbance_z);
    nh.getParam("/bluerov2_dob_node/disturbance_phi", solver_param.disturbance_phi);
    nh.getParam("/bluerov2_dob_node/disturbance_theta", solver_param.disturbance_theta);
    nh.getParam("/bluerov2_dob_node/disturbance_psi", solver_param.disturbance_psi);
    
    // 检查是否使用动态轨迹生成
    if (use_dynamic_trajectory) {
        ROS_INFO("Using dynamic rectangle trajectory generation based on current position");
        // 轨迹将在第一次收到位置信息后生成
        number_of_steps = 0;
    } else {
        // Pre-load the trajectory from file
        const char * c = REF_TRAJ.c_str();
        number_of_steps = readDataFromFile(c, trajectory);
        if (number_of_steps == 0){
            ROS_WARN("Cannot load CasADi optimal trajectory!");
        }
        else{
            ROS_INFO_STREAM("Number of steps of selected trajectory: " << number_of_steps << std::endl);
        }
    }

    // Initialize MPC
    int create_status = 1;
    create_status = bluerov2_acados_create(mpc_capsule);
    if (create_status != 0){
        ROS_INFO_STREAM("acados_create() returned status " << create_status << ". Exiting." << std::endl);
        exit(1);
    }

    // Initialize EKF
    M_values << mass + added_mass[0], mass + added_mass[1], mass + added_mass[2], Ix + added_mass[3], Iy + added_mass[4], Iz + added_mass[5];
    M = M_values.asDiagonal();
    M(0,4) = mass*ZG;
    M(1,3) = -mass*ZG;
    M(3,1) = -mass*ZG;
    M(4,0) = mass*ZG;
    invM = M.inverse();

    // Dl_values << -11.7391, -20, -31.8678, -25, -44.9085, -5;
    // Dl = Dl_values.asDiagonal();

    // K << 0.7071067811847433, 0.7071067811847433, -0.7071067811919605, -0.7071067811919605, 0.0, 0.0,
    //    0.7071067811883519, -0.7071067811883519, 0.7071067811811348, -0.7071067811811348, 0.0, 0.0,
    //    0, 0, 0, 0, 1, 1,
    //    0.051265241636155506, -0.05126524163615552, 0.05126524163563227, -0.05126524163563227, -0.11050000000000001, 0.11050000000000003,
    //    -0.05126524163589389, -0.051265241635893896, 0.05126524163641713, 0.05126524163641713, -0.002499999999974481, -0.002499999999974481,
    //    0.16652364696949604, -0.16652364696949604, -0.17500892834341342, 0.17500892834341342, 0.0, 0.0;
    

    K << 0.7071067811847431, 0.7071067811847431, -0.7071067811919607, -0.7071067811919607, 0.0, 0.0, 0.0, 0.0,
         0.7071067811883519, -0.7071067811883519, 0.7071067811811348, -0.7071067811811348, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0000000000000002, 1.0000000000000002, 1.0000000000000002, 1.0000000000000002,
         0.05126524163615552, -0.051265241636155506, 0.05126524163563227, -0.05126524163563226, -0.21805000000000005, -0.21805000000000005, 0.21805000000000005, 0.21805000000000005,
         0.05126524163589388, -0.051265241635893875, 0.051265241636417144, 0.05126524163641714, -0.1199999999999745, 0.12000000000002554, -0.11999999999997452, 0.12000000000002554,
         0.166523646969496, -0.166523646969496, -0.17500892834341342, 0.17500892834341344, 0.0, 0.0, 0.0, 0.0;
       
    Q_cov << pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,pow(dt,4)/4,
            pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),
            pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2),pow(dt,2);
    noise_Q= Q_cov.asDiagonal();
    
    esti_x << 2,0,-20,0,0,0,0,0,0,0,0,0,6,6,6,0,0,0;
    esti_P = P0;

    // Initialize body wrench force
    applied_wrench.fx = 0.0;
    applied_wrench.fy = 0.0;
    applied_wrench.fz = 0.0;
    applied_wrench.tx = 0.0;
    applied_wrench.ty = 0.0;
    applied_wrench.tz = 0.0;

    // ros subsriber & publisher
    pose_sub = nh.subscribe<nav_msgs::Odometry>("/bluerov2_heavy", 20, &BLUEROV2_DOB::pose_cb, this);
    // thrust0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/0/input",20);  // 注释掉推进器发布器
    // thrust1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/1/input",20);
    // thrust2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/2/input",20);
    // thrust3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/3/input",20);
    // thrust4_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/4/input",20);
    // thrust5_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/5/input",20);
    // thrust6_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/6/input",20);
    // thrust7_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/thrusters/7/input",20);
    ref_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/reference",20);
    error_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/mpc/error",20);
    // control_input0_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/0",20);  // 注释掉不必要的调试话题
    // control_input1_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/1",20);
    // control_input2_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/2",20);
    // control_input3_pub = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/bluerov2/control_input/3",20);    
    // esti_pose_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/ekf/pose",20);
    // esti_disturbance_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/ekf/disturbance",20);
    // applied_disturbance_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/applied_disturbance",20);
    // velocity_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/velocity",20);
    // body_velocity_pub = nh.advertise<nav_msgs::Odometry>("/bluerov2/body_velocity",20);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/bluerov2/cmd_vel",20);  // 真实机器人控制用
    subscribers.resize(8);
    for (int i = 0; i < 8; i++)
    {
        std::string topic = "/bluerov2/thrusters/" + std::to_string(i) + "/thrust";
        subscribers[i] = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(topic, 20, boost::bind(&BLUEROV2_DOB::thrusts_cb, this, _1, i));
    }
    client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/bluerov2/imu", 20, &BLUEROV2_DOB::imu_cb, this);
    
    // initialize
    for(unsigned int i=0; i < BLUEROV2_NU; i++) acados_out.u0[i] = 0.0;
    for(unsigned int i=0; i < BLUEROV2_NX; i++) acados_in.x0[i] = 0.0;
    is_start = false;
}

void BLUEROV2_DOB::pose_cb(const nav_msgs::Odometry::ConstPtr& pose)
{
    is_start = true;
    // get linear position x, y, z
    local_pos.x = pose->pose.pose.position.x;
    local_pos.y = pose->pose.pose.position.y;
    local_pos.z = pose->pose.pose.position.z;
    
    // 如果使用动态轨迹且尚未初始化，记录起始位置并生成轨迹
    if (use_dynamic_trajectory && !trajectory_initialized) {
        start_x = local_pos.x;
        start_y = local_pos.y;
        start_z = local_pos.z;
        ROS_INFO("Starting position recorded: x=%.3f, y=%.3f, z=%.3f", start_x, start_y, start_z);
        
        // 当前使用圆形轨迹（矩形轨迹已注释）
        generateDynamicCircleTrajectory();
        ROS_INFO("Dynamic circle trajectory generated with %d steps", number_of_steps);
        trajectory_initialized = true;
    }

    // get angle phi, theta, psi
    tf::quaternionMsgToTF(pose->pose.pose.orientation,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(local_euler.phi, local_euler.theta, local_euler.psi);

    // get linear velocity u, v, w
    local_pos.u = pose->twist.twist.linear.x;
    local_pos.v = pose->twist.twist.linear.y;
    local_pos.w = pose->twist.twist.linear.z;

    // get angular velocity p, q, r
    local_pos.p = pose->twist.twist.angular.x;
    local_pos.q = pose->twist.twist.angular.y;
    local_pos.r = pose->twist.twist.angular.z;

    // inertial frame velocity to body frame
    Matrix<double,3,1> v_linear_inertial;
    Matrix<double,3,1> v_angular_inertial;

    v_linear_inertial << local_pos.u, local_pos.v, local_pos.w;
    v_angular_inertial << local_pos.p, local_pos.q, local_pos.r;

    R_ib << cos(local_euler.psi)*cos(local_euler.theta), -sin(local_euler.psi)*cos(local_euler.phi)+cos(local_euler.psi)*sin(local_euler.theta)*sin(local_euler.phi), sin(local_euler.psi)*sin(local_euler.phi)+cos(local_euler.psi)*cos(local_euler.phi)*sin(local_euler.theta),
            sin(local_euler.psi)*cos(local_euler.theta), cos(local_euler.psi)*cos(local_euler.phi)+sin(local_euler.phi)*sin(local_euler.theta)*sin(local_euler.psi), -cos(local_euler.psi)*sin(local_euler.phi)+sin(local_euler.theta)*sin(local_euler.psi)*cos(local_euler.phi),
            -sin(local_euler.theta), cos(local_euler.theta)*sin(local_euler.phi), cos(local_euler.theta)*cos(local_euler.phi);
    T_ib << 1, sin(local_euler.psi)*sin(local_euler.theta)/cos(local_euler.theta), cos(local_euler.phi)*sin(local_euler.theta)/cos(local_euler.theta),
            0, cos(local_euler.phi), sin(local_euler.phi),
            0, sin(local_euler.phi)/cos(local_euler.theta), cos(local_euler.phi)/cos(local_euler.theta);
    v_linear_body = R_ib.inverse()*v_linear_inertial;
    v_angular_body = T_ib.inverse()*v_angular_inertial;

    body_acc.x = (v_linear_body[0]-pre_body_pos.u)/dt;
    body_acc.y = (v_linear_body[1]-pre_body_pos.v)/dt;
    body_acc.z = (v_linear_body[2]-pre_body_pos.w)/dt;
    body_acc.phi = (v_angular_body[0]-pre_body_pos.p)/dt;
    body_acc.theta = (v_angular_body[1]-pre_body_pos.q)/dt;
    body_acc.psi = (v_angular_body[2]-pre_body_pos.r)/dt;

    pre_body_pos.u = v_linear_body[0];
    pre_body_pos.v = v_linear_body[1];
    pre_body_pos.w = v_linear_body[2];
    pre_body_pos.p = v_angular_body[0];
    pre_body_pos.q = v_angular_body[1];
    pre_body_pos.r = v_angular_body[2];

    Matrix<double,3,1> compensate_f_inertial;
    Matrix<double,3,1> compensate_f_body;
    compensate_f_inertial << 20,0,0;
    compensate_f_body = R_ib.inverse()*compensate_f_inertial;


    }

// quaternion to euler angle
BLUEROV2_DOB::Euler BLUEROV2_DOB::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Euler euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.phi, euler.theta, euler.psi);
    return euler;
}

// euler angle to quaternion
geometry_msgs::Quaternion BLUEROV2_DOB::rpy2q(const Euler& euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.phi, euler.theta, euler.psi);
    return quaternion;
}

// read trajectory data
int BLUEROV2_DOB::readDataFromFile(const char* fileName, std::vector<std::vector<double>> &data)
{
	std::ifstream file(fileName);
	std::string line;
	int number_of_lines = 0;

	if (file.is_open())
	{
        std::cout<<"file is open"<<std::endl;
		while(getline(file, line)){
			number_of_lines++;
			std::istringstream linestream( line );
			std::vector<double> linedata;
			double number;

			while( linestream >> number ){
				linedata.push_back( number );
			}
			data.push_back( linedata );
		}

		file.close();
	}
	else
	{
        std::cout<<"file not open"<<std::endl;
		return 0;
	}

	return number_of_lines;
}
void BLUEROV2_DOB::ref_cb(int line_to_read)
{
    if (BLUEROV2_N+line_to_read+1 <= number_of_steps)  // All ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with file data
        {
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
        }
    }
    else if(line_to_read < number_of_steps)    // Part of ref points within the file
    {
        for (unsigned int i = 0; i < number_of_steps-line_to_read; i++)    // Fill part of horizon with file data
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[i+line_to_read][j];
            }
            
        }

        for (unsigned int i = number_of_steps-line_to_read; i <= BLUEROV2_N; i++)  // Fill the rest horizon with the last point
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            
        }
    }
    else    // none of ref points within the file
    {
        for (unsigned int i = 0; i <= BLUEROV2_N; i++)  // Fill all horizon with the last point
        {
            
            for (unsigned int j = 0; j <= BLUEROV2_NY; j++)
            {
                acados_in.yref[i][j] = trajectory[number_of_steps-1][j];
            }
            
        }
    }
    
}

// solve MPC
// input: current pose, reference, parameter
// output: thrust<0-5>
void BLUEROV2_DOB::solve(){
    // identify turning direction
    if (pre_yaw >= 0 && local_euler.psi >=0)
    {
        yaw_diff = local_euler.psi - pre_yaw;
    }
    else if (pre_yaw >= 0 && local_euler.psi <0)
    {
        if (2*M_PI+local_euler.psi-pre_yaw >= pre_yaw+abs(local_euler.psi))
        {
            yaw_diff = -(pre_yaw + abs(local_euler.psi));
        }
        else
        {
            yaw_diff = 2 * M_PI + local_euler.psi - pre_yaw;
        }
    }
    else if (pre_yaw < 0 && local_euler.psi >= 0)
    {
        if (2*M_PI-local_euler.psi+pre_yaw >= abs(pre_yaw)+local_euler.psi)
        {
            yaw_diff = abs(pre_yaw)+local_euler.psi;
        }
        else
        {
            yaw_diff = -(2*M_PI-local_euler.psi+pre_yaw);
        }
    }
    else
    {
        yaw_diff = local_euler.psi - pre_yaw;
    }

    yaw_sum = yaw_sum + yaw_diff;
    pre_yaw = local_euler.psi;

    // set initial states with bounds checking
    acados_in.x0[x] = std::max(-10.0, std::min(10.0, local_pos.x));  // Tank bounds safety
    acados_in.x0[y] = std::max(-10.0, std::min(10.0, local_pos.y));
    acados_in.x0[z] = std::max(-2.0, std::min(1.0, local_pos.z));
    acados_in.x0[phi] = std::max(-M_PI/4, std::min(M_PI/4, local_euler.phi));    // Limit roll/pitch
    acados_in.x0[theta] = std::max(-M_PI/4, std::min(M_PI/4, local_euler.theta));
    acados_in.x0[psi] = yaw_sum;
    acados_in.x0[u] = std::max(-2.0, std::min(2.0, v_linear_body[0]));          // Limit velocities
    acados_in.x0[v] = std::max(-2.0, std::min(2.0, v_linear_body[1]));
    acados_in.x0[w] = std::max(-2.0, std::min(2.0, v_linear_body[2]));
    acados_in.x0[p] = std::max(-2.0, std::min(2.0, v_angular_body[0]));         // Limit angular velocities
    acados_in.x0[q] = std::max(-2.0, std::min(2.0, v_angular_body[1]));
    acados_in.x0[r] = std::max(-2.0, std::min(2.0, v_angular_body[2]));
    
    // Check for NaN or infinite values
    for(int i = 0; i < 12; i++) {
        if (!std::isfinite(acados_in.x0[i])) {
            ROS_WARN_STREAM("Invalid initial state x0[" << i << "] = " << acados_in.x0[i] << ", setting to 0");
            acados_in.x0[i] = 0.0;
        }
    }
    
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, mpc_capsule->nlp_out, 0, "lbx", acados_in.x0);
    ocp_nlp_constraints_model_set(mpc_capsule->nlp_config,mpc_capsule->nlp_dims,mpc_capsule->nlp_in, mpc_capsule->nlp_out, 0, "ubx", acados_in.x0);

    // set parameters
    for (int i = 0; i < BLUEROV2_N+1; i++)
    {
        if(COMPENSATE_D == false){
            acados_param[i][0] = solver_param.disturbance_x;
            acados_param[i][1] = solver_param.disturbance_y;
            acados_param[i][2] = solver_param.disturbance_z;
            acados_param[i][3] = solver_param.disturbance_phi;
            acados_param[i][4] = solver_param.disturbance_theta;
            acados_param[i][5] = solver_param.disturbance_psi;
        }
        else if(COMPENSATE_D == true){
            acados_param[i][0] = esti_x(12)/rotor_constant;
            acados_param[i][1] = esti_x(13)/rotor_constant;
            acados_param[i][2] = esti_x(14)/rotor_constant;
            acados_param[i][3] = solver_param.disturbance_phi;
            acados_param[i][4] = solver_param.disturbance_theta;
            acados_param[i][5] = esti_x(17)/rotor_constant;
            
            // acados_param[i][0] = compensate_f_body[0]/rotor_constant;
            // acados_param[i][1] = esti_x(13)/rotor_constant;
            // acados_param[i][2] = esti_x(14)/rotor_constant;
            // acados_param[i][3] = solver_param.disturbance_phi;
            // acados_param[i][4] = solver_param.disturbance_theta;
            // acados_param[i][5] = esti_x(17)/rotor_constant;
        }
        bluerov2_acados_update_params(mpc_capsule,i,acados_param[i],BLUEROV2_NP);
    }

    // change into form of (-pi, pi)
    if(sin(acados_in.yref[0][5]) >= 0)
    {
        yaw_ref = fmod(acados_in.yref[0][5],M_PI);
    }
    else{
        yaw_ref = -M_PI + fmod(acados_in.yref[0][5],M_PI);
    }

    // set reference
    ref_cb(line_number); 
    line_number++;
    for (unsigned int i = 0; i <= BLUEROV2_N; i++){
        ocp_nlp_cost_model_set(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_in, i, "yref", acados_in.yref[i]);
    }

    // Solve OCP with error handling
    acados_status = bluerov2_acados_solve(mpc_capsule);

    if (acados_status != 0){
        ROS_WARN_STREAM("ACADOS solver failed with status " << acados_status);
        
        // Error status 3: QP solver failed, try to recover
        if (acados_status == 3) {
            ROS_WARN("QP solver failed (status 3), attempting recovery...");
            
            // Reset solver to previous solution
            static bool first_failure = true;
            if (first_failure) {
                ROS_WARN("First QP failure - using zero control inputs");
                for(unsigned int i=0; i < BLUEROV2_NU; i++) {
                    acados_out.u0[i] = 0.0;
                }
                first_failure = false;
            } else {
                ROS_WARN("Using previous control inputs");
                // Keep previous control inputs (already stored in acados_out.u0)
            }
        }
        
        // For status 1 (max iterations) or 2 (minimum step size), still use the result
        else if (acados_status == 1 || acados_status == 2) {
            ROS_WARN_STREAM("ACADOS suboptimal solution (status " << acados_status << "), using result anyway");
        }
        
        // For other errors, use zero control
        else {
            ROS_ERROR_STREAM("ACADOS critical error (status " << acados_status << "), using zero control");
            for(unsigned int i=0; i < BLUEROV2_NU; i++) {
                acados_out.u0[i] = 0.0;
            }
        }
    }

    acados_out.status = acados_status;
    acados_out.kkt_res = (double)mpc_capsule->nlp_out->inf_norm_res;

    ocp_nlp_get(mpc_capsule->nlp_solver, "time_tot", &acados_out.cpu_time);

    ocp_nlp_out_get(mpc_capsule->nlp_config, mpc_capsule->nlp_dims, mpc_capsule->nlp_out, 0, "u", (void *)acados_out.u0);
    
    // // 注释掉推进器输出，真实机器人使用cmd_vel控制
    // thrust0.data=(-acados_out.u0[0]+acados_out.u0[1]+acados_out.u0[3])/rotor_constant;
    // thrust1.data=(-acados_out.u0[0]-acados_out.u0[1]-acados_out.u0[3])/rotor_constant;
    // thrust2.data=(acados_out.u0[0]+acados_out.u0[1]-acados_out.u0[3])/rotor_constant;
    // thrust3.data=(acados_out.u0[0]-acados_out.u0[1]+acados_out.u0[3])/rotor_constant;
    // thrust4.data=(-acados_out.u0[2])/rotor_constant;
    // thrust5.data=(-acados_out.u0[2])/rotor_constant;
    // thrust6.data=(-acados_out.u0[2])/rotor_constant;
    // thrust7.data=(-acados_out.u0[2])/rotor_constant;
    
    // thrust0_pub.publish(thrust0);
    // thrust1_pub.publish(thrust1);
    // thrust2_pub.publish(thrust2);
    // thrust3_pub.publish(thrust3);
    // thrust4_pub.publish(thrust4);
    // thrust5_pub.publish(thrust5);
    // thrust6_pub.publish(thrust6);
    // thrust7_pub.publish(thrust7);

    // publish reference pose
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw_ref);
    geometry_msgs::Quaternion quat_msg;
    tf2::convert(quat, quat_msg);
    ref_pose.pose.pose.position.x = acados_in.yref[0][0];
    ref_pose.pose.pose.position.y = acados_in.yref[0][1];
    ref_pose.pose.pose.position.z = acados_in.yref[0][2];
    ref_pose.pose.pose.orientation.x = quat_msg.x;
    ref_pose.pose.pose.orientation.y = quat_msg.y;
    ref_pose.pose.pose.orientation.z = quat_msg.z;
    ref_pose.pose.pose.orientation.w = quat_msg.w;
    ref_pose.header.stamp = ros::Time::now();
    ref_pose.header.frame_id = "odom_frame";
    ref_pose.child_frame_id = "base_link";
    ref_pose_pub.publish(ref_pose);

    // publish error pose
    tf2::Quaternion quat_error;
    yaw_error = yaw_sum - acados_in.yref[0][5];
    quat_error.setRPY(0, 0, yaw_error);
    geometry_msgs::Quaternion quat_error_msg;
    tf2::convert(quat_error, quat_error_msg);
    error_pose.pose.pose.position.x = acados_in.x0[0] - acados_in.yref[0][0];
    error_pose.pose.pose.position.y = acados_in.x0[1] - acados_in.yref[0][1];
    error_pose.pose.pose.position.z = acados_in.x0[2] - acados_in.yref[0][2];
    error_pose.pose.pose.orientation.x = quat_error_msg.x;
    error_pose.pose.pose.orientation.y = quat_error_msg.y;
    error_pose.pose.pose.orientation.z = quat_error_msg.z;
    error_pose.pose.pose.orientation.w = quat_error_msg.w;
    error_pose.header.stamp = ros::Time::now();
    error_pose.header.frame_id = "odom_frame";
    error_pose.child_frame_id = "base_link";

    error_pose_pub.publish(error_pose);

    // // publish control input (注释掉不必要的调试话题)
    // control_input0.data = acados_out.u0[0];
    // control_input1.data = acados_out.u0[1];
    // control_input2.data = acados_out.u0[2];
    // control_input3.data = acados_out.u0[3];

    // control_input0_pub.publish(control_input0);
    // control_input1_pub.publish(control_input1);
    // control_input2_pub.publish(control_input2);
    // control_input3_pub.publish(control_input3);
    
    // // publish velocity in inertial frame (注释掉不必要的话题)
    // velocity_msg.header.stamp = ros::Time::now();
    // velocity_msg.header.frame_id = "odom_frame";
    // velocity_msg.child_frame_id = "base_link";
    // velocity_msg.twist.twist.linear.x = local_pos.u;   // inertial frame velocity
    // velocity_msg.twist.twist.linear.y = local_pos.v;
    // velocity_msg.twist.twist.linear.z = local_pos.w;
    // velocity_msg.twist.twist.angular.x = local_pos.p;
    // velocity_msg.twist.twist.angular.y = local_pos.q;
    // velocity_msg.twist.twist.angular.z = local_pos.r;
    // velocity_pub.publish(velocity_msg);
    
    // // publish velocity in body frame (注释掉不必要的话题)
    // body_velocity_msg.header.stamp = ros::Time::now();
    // body_velocity_msg.header.frame_id = "base_link";
    // body_velocity_msg.child_frame_id = "base_link";
    // body_velocity_msg.twist.twist.linear.x = v_linear_body[0];   // body frame velocity
    // body_velocity_msg.twist.twist.linear.y = v_linear_body[1];
    // body_velocity_msg.twist.twist.linear.z = v_linear_body[2];
    // body_velocity_msg.twist.twist.angular.x = v_angular_body[0];
    // body_velocity_msg.twist.twist.angular.y = v_angular_body[1];
    // body_velocity_msg.twist.twist.angular.z = v_angular_body[2];
    // body_velocity_pub.publish(body_velocity_msg);
    
    // Convert MPC force/torque commands to velocity commands for real robot control
    // Using simple force-to-velocity mapping (can be refined with proper dynamics)
    double force_to_vel_gain = 0.1;  // Tunable gain
    double torque_to_angvel_gain = 0.5;  // Tunable gain
    
    // MPC输出的是世界坐标系力，先转换为世界坐标系速度
    double world_vx = acados_out.u0[0] * force_to_vel_gain;   // 世界坐标系X轴速度
    double world_vy = acados_out.u0[1] * force_to_vel_gain;   // 世界坐标系Y轴速度  
    double world_vz = acados_out.u0[2] * force_to_vel_gain;   // 世界坐标系Z轴速度
    
    // 将世界坐标系速度转换为机体坐标系速度
    // 使用旋转矩阵R_ib的转置 (R_bi = R_ib^T)
    Matrix<double,3,1> world_velocity;
    Matrix<double,3,1> body_velocity;
    
    world_velocity << world_vx, world_vy, world_vz;
    body_velocity = R_ib.transpose() * world_velocity;  // R_ib^T * v_world = v_body
    
    // 发布机体坐标系速度命令
    cmd_vel_msg.linear.x = body_velocity(0);    // 机体X轴速度（前进/后退）
    cmd_vel_msg.linear.y = body_velocity(1);    // 机体Y轴速度（左右平移）
    cmd_vel_msg.linear.z = body_velocity(2);    // 机体Z轴速度（上下运动）
    cmd_vel_msg.angular.x = 0.0;                // 不控制横滚
    cmd_vel_msg.angular.y = 0.0;                // 不控制俯仰
    cmd_vel_msg.angular.z = acados_out.u0[3] * torque_to_angvel_gain; // 偏航角速度
    
    cmd_vel_pub.publish(cmd_vel_msg);
    
}

void BLUEROV2_DOB::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // get linear position x, y, z
    local_acc.x = round(msg->linear_acceleration.x*10000)/10000;
    local_acc.y = round(msg->linear_acceleration.y*10000)/10000;
    local_acc.z = round(msg->linear_acceleration.z*10000)/10000-g;
    
}

void BLUEROV2_DOB::thrusts_cb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg, int index)
{
    double input = msg->data;
    //ROS_INFO("Received input for thruster %d: %f", index, input);
    switch (index)
    {
        case 0:
            current_t.t0 = input;
            break;
        case 1:
            current_t.t1 = input;
            break;
        case 2:
            current_t.t2 = input;
            break;
        case 3:
            current_t.t3 = input;
            break;
        case 4:
            current_t.t4 = input;
            break;
        case 5:
            current_t.t5 = input;
            break;
        case 6:
            current_t.t6 = input;
            break;
        case 7:
            current_t.t7 = input;
            break;
        default:
            ROS_WARN("Invalid thruster index: %d", index);
            break;
    }
}

// Define EKF function
// inputs: current state estimate x, current covariance estimate P, input u, measurement y, 
//        process noise covariance Q, measuremnt noise covariance R
void BLUEROV2_DOB::EKF()
{
    // std::cout<<"esti_x12:    " << esti_x(12) << std::endl;
    // get input u and measuremnet y
    meas_u << current_t.t0, current_t.t1, current_t.t2, current_t.t3, current_t.t4, current_t.t5,current_t.t6,current_t.t7;
    Matrix<double,6,1> tau;
    tau = K*meas_u;
    meas_y << local_pos.x, local_pos.y, local_pos.z, local_euler.phi, local_euler.theta, local_euler.psi,
            v_linear_body[0], v_linear_body[1], v_linear_body[2], v_angular_body[0], v_angular_body[1], v_angular_body[2],
            tau(0),tau(1),tau(2),tau(3),tau(4),tau(5);
    
    // Define Jacobian matrices of system dynamics and measurement model
    Matrix<double,18,18> F;     // Jacobian of system dynamics
    Matrix<double,18,18> H;     // Jacobian of measurement model

    // Define Kalman gain matrix
    Matrix<double,18,18> Kal;

    // Define prediction and update steps
    Matrix<double,18,1> x_pred;     // predicted state
    Matrix<double,18,18> P_pred;    // predicted covariance
    Matrix<double,18,1> y_pred;     // predicted measurement
    Matrix<double,18,1> y_err;      // measurement error
    
    // Prediction step: estimate state and covariance at time k+1|k
    F = compute_jacobian_F(esti_x, meas_u);             // compute Jacobian of system dynamics at current state and input
    x_pred = RK4(esti_x, meas_u);                       // predict state at time k+1|k
    // dx = f(esti_x, meas_u);                             // acceleration
    P_pred = F * esti_P * F.transpose() + noise_Q;      // predict covariance at time k+1|k
    
    // Update step: correct state and covariance using measurement at time k+1
    H = compute_jacobian_H(x_pred);                         // compute Jacobian of measurement model at predicted state
    y_pred = h(x_pred);                                     // predict measurement at time k+1
    y_err = meas_y - y_pred;                                // compute measurement error
    Kal = P_pred * H.transpose() * (H * P_pred * H.transpose() + noise_R).inverse();    // compute Kalman gain
    esti_x = x_pred + Kal * y_err;                          // correct state estimate
    esti_P = (MatrixXd::Identity(n, n) - Kal * H) * P_pred * (MatrixXd::Identity(n, n) - Kal * H).transpose() + Kal*noise_R*Kal.transpose(); // correct covariance estimate
    
    // body frame disturbance to inertial frame
    wf_disturbance << (cos(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (-sin(meas_y(5))*cos(meas_y(3))+cos(meas_y(5))*sin(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (sin(meas_y(5))*sin(meas_y(3))+cos(meas_y(5))*cos(meas_y(3))*sin(meas_y(4)))*esti_x(14),
            (sin(meas_y(5))*cos(meas_y(4)))*esti_x(12) + (cos(meas_y(5))*cos(meas_y(3))+sin(meas_y(3))*sin(meas_y(4))*sin(meas_y(5)))*esti_x(13) + (-cos(meas_y(5))*sin(meas_y(3))+sin(meas_y(4))*sin(meas_y(5))*cos(meas_y(3)))*esti_x(14),
            (-sin(meas_y(4)))*esti_x(12) + (cos(meas_y(4))*sin(meas_y(3)))*esti_x(13) + (cos(meas_y(4))*cos(meas_y(3)))*esti_x(14),
            esti_x(15) + (sin(meas_y(5))*sin(meas_y(4))/cos(meas_y(4)))*esti_x(16) + cos(meas_y(3))*sin(meas_y(4))/cos(meas_y(4))*esti_x(17),
            (cos(meas_y(3)))*esti_x(16) + (sin(meas_y(3)))*esti_x(17),
            (sin(meas_y(3))/cos(meas_y(4)))*esti_x(16) + (cos(meas_y(3))/cos(meas_y(4)))*esti_x(17);
    
    // // publish estimate pose (注释掉不必要的话题)
    // tf2::Quaternion quat;
    // quat.setRPY(esti_x(3), esti_x(4), esti_x(5));
    // geometry_msgs::Quaternion quat_msg;
    // tf2::convert(quat, quat_msg);
    // esti_pose.pose.pose.position.x = esti_x(0);
    // esti_pose.pose.pose.position.y = esti_x(1);
    // esti_pose.pose.pose.position.z = esti_x(2);
    // esti_pose.pose.pose.orientation.x = quat_msg.x;
    // esti_pose.pose.pose.orientation.y = quat_msg.y;
    // esti_pose.pose.pose.orientation.z = quat_msg.z;
    // esti_pose.pose.pose.orientation.w = quat_msg.w;
    // esti_pose.twist.twist.linear.x = esti_x(6);
    // esti_pose.twist.twist.linear.y = esti_x(7);
    // esti_pose.twist.twist.linear.z = esti_x(8);
    // esti_pose.twist.twist.angular.x = esti_x(9);
    // esti_pose.twist.twist.angular.y = esti_x(10);
    // esti_pose.twist.twist.angular.z = esti_x(11);
    // esti_pose.header.stamp = ros::Time::now();
    // esti_pose.header.frame_id = "odom_frame";
    // esti_pose.child_frame_id = "base_link";
    // esti_pose_pub.publish(esti_pose);

    // // publish estimate disturbance (注释掉不必要的话题)
    // esti_disturbance.pose.pose.position.x = wf_disturbance(0);
    // esti_disturbance.pose.pose.position.y = wf_disturbance(1);
    // esti_disturbance.pose.pose.position.z = wf_disturbance(2);
    // esti_disturbance.twist.twist.angular.x = wf_disturbance(3);
    // esti_disturbance.twist.twist.angular.y = wf_disturbance(4);
    // esti_disturbance.twist.twist.angular.z = wf_disturbance(5);
    // esti_disturbance.header.stamp = ros::Time::now();
    // esti_disturbance.header.frame_id = "odom_frame";
    // esti_disturbance.child_frame_id = "base_link";
    // esti_disturbance_pub.publish(esti_disturbance);

    // // publish applied disturbance (注释掉不必要的话题)
    // applied_disturbance.pose.pose.position.x = applied_wrench.fx;
    // applied_disturbance.pose.pose.position.y = applied_wrench.fy;
    // applied_disturbance.pose.pose.position.z = applied_wrench.fz;
    // applied_disturbance.twist.twist.angular.x = applied_wrench.tx;
    // applied_disturbance.twist.twist.angular.y = applied_wrench.ty;
    // applied_disturbance.twist.twist.angular.z = applied_wrench.tz;
    // applied_disturbance.header.stamp = ros::Time::now();
    // applied_disturbance.header.frame_id = "odom_frame";
    // applied_disturbance.child_frame_id = "base_link";
    // applied_disturbance_pub.publish(applied_disturbance);

    // print estimate disturbance and velocity information
    if(cout_counter > 2){
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        // Thrust to force/moment conversion information
        Matrix<double,6,1> tau = K*meas_u;
        std::cout << "=== THRUST TO VELOCITY CONVERSION ===" << std::endl;
        std::cout << "Individual Thrusts: t0=" << current_t.t0 << " t1=" << current_t.t1 << " t2=" << current_t.t2 << " t3=" << current_t.t3 
                  << " t4=" << current_t.t4 << " t5=" << current_t.t5 << " t6=" << current_t.t6 << " t7=" << current_t.t7 << std::endl;
        std::cout << "Generated Forces/Moments: Fx=" << tau(0) << " Fy=" << tau(1) << " Fz=" << tau(2) 
                  << " Mx=" << tau(3) << " My=" << tau(4) << " Mz=" << tau(5) << std::endl;
        std::cout << "Body Frame Velocities: u=" << v_linear_body[0] << " v=" << v_linear_body[1] << " w=" << v_linear_body[2] 
                  << " p=" << v_angular_body[0] << " q=" << v_angular_body[1] << " r=" << v_angular_body[2] << std::endl;
        std::cout << "Inertial Frame Velocities: u=" << local_pos.u << " v=" << local_pos.v << " w=" << local_pos.w 
                  << " p=" << local_pos.p << " q=" << local_pos.q << " r=" << local_pos.r << std::endl;
        std::cout << "Body Frame Accelerations: u_dot=" << body_acc.x << " v_dot=" << body_acc.y << " w_dot=" << body_acc.z 
                  << " p_dot=" << body_acc.phi << " q_dot=" << body_acc.theta << " r_dot=" << body_acc.psi << std::endl;
        
        std::cout << "\n=== CONTROL AND REFERENCE ===" << std::endl;
        std::cout << "MPC Control Inputs: u0=" << acados_out.u0[0] << " u1=" << acados_out.u0[1] << " u2=" << acados_out.u0[2] << " u3=" << acados_out.u0[3] << std::endl;
        
        // 显示坐标系转换信息
        double world_vx_debug = acados_out.u0[0] * 0.1;
        double world_vy_debug = acados_out.u0[1] * 0.1;
        double world_vz_debug = acados_out.u0[2] * 0.1;
        std::cout << "World Frame Velocity: vx=" << world_vx_debug << " vy=" << world_vy_debug << " vz=" << world_vz_debug << std::endl;
        std::cout << "Body Frame Velocity: vx=" << cmd_vel_msg.linear.x << " vy=" << cmd_vel_msg.linear.y << " vz=" << cmd_vel_msg.linear.z << " wz=" << cmd_vel_msg.angular.z << std::endl;
        std::cout << "Current Yaw Angle: " << local_euler.psi << " rad (" << local_euler.psi * 180.0 / M_PI << " deg)" << std::endl;
        std::cout << "tau_x:  " << meas_y(12) << "  tau_y:  " << meas_y(13) << "  tau_z:  " << meas_y(14) << "  tau_psi:  " << meas_y(17) << std::endl;
        std::cout << "ref_x:    " << acados_in.yref[0][0] << "\tref_y:   " << acados_in.yref[0][1] << "\tref_z:    " << acados_in.yref[0][2] << "\tref_yaw:    " << yaw_ref << std::endl;
        std::cout << "pos_x: " << meas_y(0) << "  pos_y: " << meas_y(1) << "  pos_z: " << meas_y(2) << " phi: " << meas_y(3) << "  theta: " << meas_y(4) << "  psi: " << meas_y(5) <<std::endl;
        std::cout << "esti_x: " << esti_x(0) << "  esti_y: " << esti_x(1) << "  esti_z: " << esti_x(2) << " esti_phi: " << esti_x(3) << "  esti_theta: " << esti_x(4) << "  esti_psi: " << esti_x(5) <<std::endl;
        std::cout << "error_x:  " << error_pose.pose.pose.position.x << "  error_y:  " << error_pose.pose.pose.position.y << "  error_z:  " << error_pose.pose.pose.position.z << std::endl;
        
        std::cout << "\n=== DISTURBANCES ===" << std::endl;
        std::cout << "applied force x:  " << applied_wrench.fx << "\tforce y:  " << applied_wrench.fy << "\tforce_z:  " << applied_wrench.fz << std::endl;
        std::cout << "applied torque x:  " << applied_wrench.tx << "\ttorque y:  " << applied_wrench.ty << "\ttorque_z:  " << applied_wrench.tz << std::endl;
        std::cout << "(body frame) disturbance x: " << esti_x(12) << "    disturbance y: " << esti_x(13) << "    disturbance z: " << esti_x(14) << std::endl;
        std::cout << "(world frame) disturbance x: " << wf_disturbance(0) << "    disturbance y: " << wf_disturbance(1) << "    disturbance z: " << wf_disturbance(2) << std::endl;
        std::cout << "(world frame) disturbance phi: " << wf_disturbance(3) << "    disturbance theta: " << wf_disturbance(4) << "    disturbance psi: " << wf_disturbance(5) << std::endl;
        
        std::cout << "\n=== SOLVER INFO ===" << std::endl;
        std::cout << "solve_time: "<< acados_out.cpu_time << "\tkkt_res: " << acados_out.kkt_res << "\tacados_status: " << acados_out.status << std::endl;
        std::cout << "ros_time:   " << std::fixed << ros::Time::now().toSec() << std::endl;
        std::cout << "---------------------------------------------------------------------------------------------------------------------" << std::endl;
        cout_counter = 0;
    }
    else{
        cout_counter++;
    }
}

// 4th order RK for integration
MatrixXd BLUEROV2_DOB::RK4(MatrixXd x, MatrixXd u)
{
    Matrix<double,18,1> k1;
    Matrix<double,18,1> k2;
    Matrix<double,18,1> k3;
    Matrix<double,18,1> k4;

    k1 = f(x, u) * dt;
    k2 = f(x+k1/2, u) * dt;
    k3 = f(x+k2/3, u) * dt;
    k4 = f(x+k3, u) * dt;

    return x + (k1+2*k2+2*k3+k4)/6;
}

// Define system dynamics function
MatrixXd BLUEROV2_DOB::f(MatrixXd x, MatrixXd u)
{
    // Define system dynamics
    Matrix<double,18,1> xdot;

    KAu = K*u;
    xdot << (cos(x(5))*cos(x(4)))*x(6) + (-sin(x(5))*cos(x(3))+cos(x(5))*sin(x(4))*sin(x(3)))*x(7) + (sin(x(5))*sin(x(3))+cos(x(5))*cos(x(3))*sin(x(4)))*x(8),  //xdot
            (sin(x(5))*cos(x(4)))*x(6) + (cos(x(5))*cos(x(3))+sin(x(3))*sin(x(4))*sin(x(5)))*x(7) + (-cos(x(5))*sin(x(3))+sin(x(4))*sin(x(5))*cos(x(3)))*x(8),
            (-sin(x(4)))*x(6) + (cos(x(4))*sin(x(3)))*x(7) + (cos(x(4))*cos(x(3)))*x(8),
            x(9) + (sin(x(5))*sin(x(4))/cos(x(4)))*x(10) + cos(x(3))*sin(x(4))/cos(x(4))*x(11),
            (cos(x(3)))*x(10) + (sin(x(3)))*x(11),
            (sin(x(3))/cos(x(4)))*x(10) + (cos(x(3))/cos(x(4)))*x(11), 
            invM(0,0)*(KAu(0)+mass*x(11)*x(7)-mass*x(10)*x(8)-bouyancy*sin(x(4))+x(12)+Dl[0]*x(6)+Dnl[0]*abs(x(6))*x(6)),    // xddot: M^-1[tau+w-C-g-D]
            invM(1,1)*(KAu(1)-mass*x(11)*x(6)+mass*x(9)*x(8)+bouyancy*cos(x(4))*sin(x(3))+x(13)+Dl[1]*x(7)+Dnl[1]*abs(x(7))*x(7)),
            invM(2,2)*(KAu(2)+mass*x(10)*x(6)-mass*x(9)*x(7)+bouyancy*cos(x(4))*cos(x(3))+x(14)+Dl[2]*x(8)+Dnl[2]*abs(x(8))*x(8)),
            invM(3,3)*(KAu(3)+(Iy-Iz)*x(10)*x(11)-mass*ZG*g*cos(x(4))*sin(x(3))+x(15)+Dl[3]*x(9)+Dnl[3]*abs(x(9))*x(9)),
            invM(4,4)*(KAu(4)+(Iz-Ix)*x(9)*x(11)-mass*ZG*g*sin(x(4))+x(16)+Dl[4]*x(10)+Dnl[4]*abs(x(10))*x(10)),
            invM(5,5)*(KAu(5)-(Iy-Ix)*x(9)*x(10)+x(17)+Dl[5]*x(11)+Dnl[5]*abs(x(11))*x(11)),
            // invM(0,0)*(KAu(0)+mass*x(11)*x(7)-mass*x(10)*x(8)-bouyancy*sin(x(4))+x(12)+Dl(0,0)*x(6)+added_mass[2]*x(2)*x(4)),    // xddot: M^-1[tau+w-C-g-D]
            // invM(1,1)*(KAu(1)-mass*x(11)*x(6)+mass*x(9)*x(8)+bouyancy*cos(x(4))*sin(x(3))+x(13)+Dl(1,1)*x(7)-added_mass[2]*x(2)*x(3)-added_mass[0]*x(0)*x(5)),
            // invM(2,2)*(KAu(2)+mass*x(10)*x(6)-mass*x(9)*x(7)+bouyancy*cos(x(4))*cos(x(3))+x(14)+Dl(2,2)*x(8)-added_mass[1]*x(1)*x(3)+added_mass[0]*x(0)*x(4)),
            // invM(3,3)*(KAu(3)+(Iy-Iz)*x(10)*x(11)-mass*ZG*g*cos(x(4))*sin(x(3))+x(15)+Dl(3,3)*x(9)-added_mass[2]*x(2)*x(1)+added_mass[1]*x(1)*x(2)-added_mass[5]*x(5)*x(4)+added_mass[4]*x(4)*x(5)),
            // invM(4,4)*(KAu(4)+(Iz-Ix)*x(9)*x(11)-mass*ZG*g*sin(x(4))+x(16)+Dl(4,4)*x(10)+added_mass[2]*x(2)*x(0)-added_mass[0]*x(0)*x(2)+added_mass[5]*x(5)*x(3)-added_mass[3]*x(3)*x(5)),
            // invM(5,5)*(KAu(5)-(Iy-Ix)*x(9)*x(10)+x(17)+Dl(5,5)*x(11)-added_mass[1]*x(1)*x(0)+added_mass[0]*x(0)*x(1)-added_mass[4]*x(4)*x(3)+added_mass[3]*x(3)*x(4)),
            0,0,0,0,0,0;
            
    return xdot; // dt is the time step
}

// Define measurement model function (Z = Hx, Z: measurement vector [x,xdot,tau]; X: state vector [x,xdot,disturbance])
MatrixXd BLUEROV2_DOB::h(MatrixXd x)
{
    // Define measurement model
    Matrix<double,18,1> y;
    y << x(0),x(1),x(2),x(3),x(4),x(5),
        x(6),x(7),x(8),x(9),x(10),x(11),
        M(0,0)*body_acc.x-mass*x(11)*x(7)+mass*x(10)*x(8)+bouyancy*sin(x(4))-x(12)-Dl[0]*x(6)-Dnl[0]*abs(x(6))*x(6),        
        M(1,1)*body_acc.y+mass*x(11)*x(6)-mass*x(9)*x(8)-bouyancy*cos(x(4))*sin(x(3))-x(13)-Dl[1]*x(7)-Dnl[1]*abs(x(7))*x(7),
        M(2,2)*body_acc.z-mass*x(10)*x(6)+mass*x(9)*x(7)-bouyancy*cos(x(4))*cos(x(3))-x(14)-Dl[2]*x(8)-Dnl[2]*abs(x(8))*x(8),
        M(3,3)*body_acc.phi-(Iy-Iz)*x(10)*x(11)+mass*ZG*g*cos(x(4))*sin(x(3))-x(15)-Dl[3]*x(9)-Dnl[3]*abs(x(9))*x(9),
        M(4,4)*body_acc.theta-(Iz-Ix)*x(9)*x(11)+mass*ZG*g*sin(x(4))-x(16)-Dl[4]*x(10)-Dnl[4]*abs(x(10))*x(10),
        M(5,5)*body_acc.psi+(Iy-Ix)*x(9)*x(10)-x(17)-Dl[5]*x(11)-Dnl[5]*abs(x(11))*x(11);
        // M(0,0)*body_acc.x-mass*x(11)*x(7)+mass*x(10)*x(8)+bouyancy*sin(x(4))-x(12)-Dl(0,0)*x(6)-added_mass[2]*x(2)*x(4),        
        // M(1,1)*body_acc.y+mass*x(11)*x(6)-mass*x(9)*x(8)-bouyancy*cos(x(4))*sin(x(3))-x(13)-Dl(1,1)*x(7)+added_mass[2]*x(2)*x(3)+added_mass[0]*x(0)*x(5),
        // M(2,2)*body_acc.z-mass*x(10)*x(6)+mass*x(9)*x(7)-bouyancy*cos(x(4))*cos(x(3))-x(14)-Dl(2,2)*x(8)+added_mass[1]*x(1)*x(3)-added_mass[0]*x(0)*x(4),
        // M(3,3)*body_acc.phi-(Iy-Iz)*x(10)*x(11)+mass*ZG*g*cos(x(4))*sin(x(3))-x(15)-Dl(3,3)*x(9)+added_mass[2]*x(2)*x(1)-added_mass[1]*x(1)*x(2)+added_mass[5]*x(5)*x(4)-added_mass[4]*x(4)*x(5),
        // M(4,4)*body_acc.theta-(Iz-Ix)*x(9)*x(11)+mass*ZG*g*sin(x(4))-x(16)-Dl(4,4)*x(10)-added_mass[2]*x(2)*x(0)+added_mass[0]*x(0)*x(2)-added_mass[5]*x(5)*x(3)+added_mass[3]*x(3)*x(5),
        // M(5,5)*body_acc.psi+(Iy-Ix)*x(9)*x(10)-x(17)-Dl(5,5)*x(11)+added_mass[1]*x(1)*x(0)-added_mass[0]*x(0)*x(1)+added_mass[4]*x(4)*x(3)-added_mass[3]*x(3)*x(4);

    return y;
}

// Define function to compute Jacobian of system dynamics at current state and input
MatrixXd BLUEROV2_DOB::compute_jacobian_F(MatrixXd x, MatrixXd u)
{
    // Define Jacobian of system dynamics
    Matrix<double,18,18> F;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = RK4(x, u);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = RK4(x1, u);
        F.col(i) = (f1-f0)/d;
    }
    return F;
}

// Define function to compute Jacobian of measurement model at predicted state
MatrixXd BLUEROV2_DOB::compute_jacobian_H(MatrixXd x)
{
    // Define Jacobian of measurement model
    Matrix<double,18,18> H;
    double d = 1e-6;                    // finite difference step size
    VectorXd f0 = h(x);
    for (int i = 0; i < n; i++){
        VectorXd x1 = x;
        x1(i) += d;
        VectorXd f1 = h(x1);
        H.col(i) = (f1-f0)/d;
    }
    return H;
}

void BLUEROV2_DOB::applyBodyWrench()
{
    // initialize periodic disturbance
    // double amplitudeScalingFactor;

    // initialize random disturbance
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> distribution(0.5, 1.0);

    // initialize disturbance data from file
    std::vector<double> data_fx;
    std::vector<double> data_fy;
    std::vector<double> data_fz;
    std::vector<double> data_tz;
    const char * fx_c = WRENCH_FX.c_str();
    const char * fy_c = WRENCH_FY.c_str();
    const char * fz_c = WRENCH_FZ.c_str();
    const char * tz_c = WRENCH_TZ.c_str();

    if(READ_WRENCH == 0){
        // generate periodical disturbance
        if(dis_time > periodic_counter*M_PI)
        {
            amplitudeScalingFactor_X = distribution(gen)*6;
            amplitudeScalingFactor_Y = distribution(gen)*6;
            amplitudeScalingFactor_Z = distribution(gen)*6;
            amplitudeScalingFactor_N = distribution(gen)*6;
            periodic_counter++;
        }
        applied_wrench.fx = sin(dis_time)*amplitudeScalingFactor_X;
        applied_wrench.fy = sin(dis_time)*amplitudeScalingFactor_Y;
        applied_wrench.fz = sin(dis_time)*amplitudeScalingFactor_Z;
        applied_wrench.tz = (sin(dis_time)*amplitudeScalingFactor_Y)/3;
        if(dis_time>10){
            applied_wrench.fx = applied_wrench.fx;
            applied_wrench.fy = applied_wrench.fy;
            applied_wrench.fz = applied_wrench.fz;
            applied_wrench.tz = applied_wrench.tz;
        }

        dis_time = dis_time+dt*2.5;
        // std::cout << "amplitudeScalingFactor_Z:  " << amplitudeScalingFactor_Z << "  amplitudeScalingFactor_N:  " << amplitudeScalingFactor_N << std::endl;
    }
    else if(READ_WRENCH == 1){
        // generate random disturbance
        // if(rand_counter > 10){
        //     applied_wrench.fx = distribution(gen)*5;
        //     applied_wrench.fy = distribution(gen)*5;
        //     applied_wrench.fz = distribution(gen)*5;
        //     applied_wrench.tx = distribution(gen);
        //     applied_wrench.ty = distribution(gen);
        //     applied_wrench.tz = distribution(gen);
        //     rand_counter = 0;
        // }
        // else{
        //     rand_counter++;
        // }
        // generate constant disturbance
        applied_wrench.fx = 10;
        applied_wrench.fy = 10;
        applied_wrench.fz = 10;
        applied_wrench.tz = 0;
    }
    else if(READ_WRENCH == 2){
        // std::cout << "read from file starts" << std::endl;
        // read disturbance from file
        // read force x
        std::ifstream fx_file(fx_c);
        if (!fx_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_fx;
        while (fx_file >> value_fx) {
            data_fx.push_back(value_fx); // Load the data into the vector
        }
        fx_file.close();

        // read force y
        std::ifstream fy_file(fy_c);
        if (!fy_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_fy;
        while (fy_file >> value_fy) {
            data_fy.push_back(value_fy); // Load the data into the vector
        }
        fy_file.close();

        // read force z
        std::ifstream fz_file(fz_c);
        if (!fz_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_fz;
        while (fz_file >> value_fz) {
            data_fz.push_back(value_fz); // Load the data into the vector
        }
        fz_file.close();

        // read torque z
        std::ifstream tz_file(tz_c);
        if (!tz_file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }
        double value_tz;
        while (tz_file >> value_tz) {
            data_tz.push_back(value_tz); // Load the data into the vector
        }
        tz_file.close();

        applied_wrench.fx  = data_fx[fx_counter];
        applied_wrench.fy  = data_fy[fx_counter];
        applied_wrench.fz  = data_fz[fx_counter];
        applied_wrench.tz  = data_tz[fx_counter];
        fx_counter++;
    }
    
    // call ros service apply_body_wrench
    body_wrench.request.body_name = "bluerov2/base_link";
    body_wrench.request.start_time = ros::Time(0.0);
    body_wrench.request.reference_frame = "world";
    body_wrench.request.duration = ros::Duration(1000);
    body_wrench.request.reference_point.x = 0.0;
    body_wrench.request.reference_point.y = 0.0;
    body_wrench.request.reference_point.z = 0.0;
    body_wrench.request.wrench.force.x = applied_wrench.fx;
    body_wrench.request.wrench.force.y = applied_wrench.fy;
    body_wrench.request.wrench.force.z = applied_wrench.fz;
    body_wrench.request.wrench.torque.x = applied_wrench.tx;
    body_wrench.request.wrench.torque.y = applied_wrench.ty;
    body_wrench.request.wrench.torque.z = applied_wrench.tz;
    client.call(body_wrench);
    
}

// coriolis and centripetal forces C(v) = C_RB(v) + C_A(v)
// v(0-5):u, v, w, p, q, r
MatrixXd BLUEROV2_DOB::dynamics_C(MatrixXd v)
{
    Matrix<double,6,6> C;
    C<< 0, 0, 0, 0, mass*v(2)+added_mass[2]*v(2), -mass*v(1)+added_mass[1]*v(1),
        0, 0, 0, -mass*v(2)-added_mass[2]*v(2), 0, mass*v(0)-added_mass[0]*v(0),
        0, 0, 0, mass*v(1)-added_mass[1]*v(1), -mass*v(0)+added_mass[0]*v(0), 0,
        0, mass*v(2)-added_mass[2]*v(2), -mass*v(1)+added_mass[1]*v(1), 0, Iz*v(5)-added_mass[5]*v(5), -Iy*v(4)+added_mass[4]*v(4),
        -mass*v(2)+added_mass[2]*v(2), 0, mass*v(0)-added_mass[0]*v(0), -Iz*v(5)+added_mass[5]*v(5), 0, Ix*v(3)-added_mass[3]*v(3),
        mass*v(1)-added_mass[1]*v(1), -mass*v(0)+added_mass[0]*v(0), 0, Iy*v(4)-added_mass[4]*v(4), -Ix*v(3)+added_mass[3]*v(3), 0;
    return C;
}

// damping forces D(v) = D_L + D_NL(v)
// v(0-5):u, v, w, p, q, r
MatrixXd BLUEROV2_DOB::dynamics_D(MatrixXd v)
{
    Matrix<double,1,6> D_diagonal;
    D_diagonal << -Dl[0]-Dnl[0]*abs(v(0)), -Dl[1]-Dnl[1]*abs(v(1)), -Dl[2]-Dnl[2]*abs(v(2)),
                -Dl[3]-Dnl[3]*abs(v(3)), -Dl[4]-Dnl[4]*abs(v(4)), -Dl[5]-Dnl[5]*abs(v(5));

    Matrix<double,6,6> D;
    D = D_diagonal.asDiagonal();

    return D;
}

// gravitational and buoyancy forces g
// euler(0-2): phi, theta, psi
MatrixXd BLUEROV2_DOB::dynamics_g(MatrixXd euler)
{
    Matrix<double,6,1> g;

    g << bouyancy*sin(euler(1)),
        -bouyancy*cos(euler(1))*sin(euler(0)),
        -bouyancy*cos(euler(1))*cos(euler(0)),
        mass*ZG*g*cos(euler(1))*sin(euler(0)),
        mass*ZG*g*sin(euler(1)),
        0;

    return g;
}

// 动态生成矩形轨迹从的
// 动态生成矩形轨迹 (已注释，当前使用圆形轨迹)
/*
void BLUEROV2_DOB::generateDynamicRectangleTrajectory()
{
    // 矩形轨迹参数
    double rect_width = 1.0;   // Y轴正方向移动距离 (m)
    double rect_height = 0.3;  // X轴正方向移动距离 (m)
    
    // 计算矩形轨迹的四个顶点（世界坐标系）
    std::vector<std::array<double, 3>> waypoints = {
        {start_x, start_y, start_z},                    // 起始点
        {start_x, start_y + rect_width, start_z},       // Y轴正方向移动2m
        {start_x + rect_height, start_y + rect_width, start_z}, // X轴正方向移动0.5m  
        {start_x + rect_height, start_y, start_z},      // Y轴负方向移动2m
        {start_x, start_y, start_z}                     // X轴负方向移动0.5m，回到起始点
    };
    
    // 计算每段的长度和所需时间
    std::vector<double> segment_lengths;
    std::vector<double> segment_durations;
    double total_duration = 0;
    
    for (size_t i = 0; i < waypoints.size() - 1; i++) {
        double dx = waypoints[i+1][0] - waypoints[i][0];
        double dy = waypoints[i+1][1] - waypoints[i][1];
        double length = sqrt(dx*dx + dy*dy);
        double duration = length / rectangle_speed;
        
        segment_lengths.push_back(length);
        segment_durations.push_back(duration);
        total_duration += duration;
        
        ROS_INFO("Segment %zu: length=%.3fm, duration=%.1fs", i+1, length, duration);
    }
    
    // 生成轨迹点
    trajectory.clear();
    double current_time = 0;
    
    for (size_t seg = 0; seg < waypoints.size() - 1; seg++) {
        double seg_duration = segment_durations[seg];
        int seg_points = (int)(seg_duration / sample_time);
        
        for (int i = 0; i <= seg_points; i++) {
            double t_norm = (double)i / seg_points; // [0, 1]
            if (i == seg_points && seg < waypoints.size() - 2) {
                continue; // 避免重复添加分界点（除了最后一个）
            }
            
            // 线性插值计算当前位置
            double x = waypoints[seg][0] + t_norm * (waypoints[seg+1][0] - waypoints[seg][0]);
            double y = waypoints[seg][1] + t_norm * (waypoints[seg+1][1] - waypoints[seg][1]);
            double z = waypoints[seg][2]; // Z保持不变
            
            // 计算速度（方向向量 × 速度大小）
            double dx = waypoints[seg+1][0] - waypoints[seg][0];
            double dy = waypoints[seg+1][1] - waypoints[seg][1];
            double seg_length = segment_lengths[seg];
            double vx = (seg_length > 0) ? (dx / seg_length) * rectangle_speed : 0.0;
            double vy = (seg_length > 0) ? (dy / seg_length) * rectangle_speed : 0.0;
            double vz = 0.0;
            
            // 创建轨迹点 (16列格式)
            std::vector<double> traj_point(16, 0.0);
            traj_point[0] = x;      // x
            traj_point[1] = y;      // y  
            traj_point[2] = z;      // z
                     traj_point[3] = 0.0;        // phi (roll)
         traj_point[4] = 0.0;        // theta (pitch)
         traj_point[5] = yaw_angle;  // psi (yaw) - 朝向运动方向
         traj_point[6] = body_vx;    // u (机体x velocity - 前进)
         traj_point[7] = body_vy;    // v (机体y velocity - 侧移)
         traj_point[8] = body_vz;    // w (机体z velocity - 垂直)
            traj_point[9] = 0.0;    // p (roll rate)
            traj_point[10] = 0.0;   // q (pitch rate)
            traj_point[11] = 0.0;   // r (yaw rate)
            traj_point[12] = 0.0;   // u1 (control input)
            traj_point[13] = 0.0;   // u2
            traj_point[14] = 0.0;   // u3
            traj_point[15] = 0.0;   // u4
            
            trajectory.push_back(traj_point);
            current_time += sample_time;
        }
    }
    
    // 添加额外的点用于MPC预测视野
    if (!trajectory.empty()) {
        std::vector<double> last_point = trajectory.back();
        // 将最后一点的速度设为0（停止状态）
        last_point[6] = 0.0; // vx
        last_point[7] = 0.0; // vy
        last_point[8] = 0.0; // vz
        
        for (int i = 0; i < 20; i++) {
            trajectory.push_back(last_point);
        }
    }
    
    number_of_steps = trajectory.size();
    
    ROS_INFO("Generated dynamic rectangle trajectory:");
    ROS_INFO("- Start: (%.3f, %.3f, %.3f)", start_x, start_y, start_z);
    ROS_INFO("- Rectangle size: %.1fm(Y) x %.1fm(X)", rect_width, rect_height);
    ROS_INFO("- Speed: %.3f m/s", rectangle_speed);
    ROS_INFO("- Total duration: %.1f seconds", total_duration);
    ROS_INFO("- Number of points: %d", number_of_steps);
}
*/

// 动态生成圆形轨迹
void BLUEROV2_DOB::generateDynamicCircleTrajectory()
{
    // 简化的圆形轨迹参数
    double radius = 0.8;                    // 固定半径 0.8m
    double speed = 0.2;                     // 固定速度 0.2m/s
    double angular_velocity = speed / radius;        // 角速度 (rad/s)
    double period = 2 * M_PI / angular_velocity;     // 一圈所需时间 (s)
    
    // 圆心位置（起始位置向Y轴正方向偏移半径距离）
    double center_x = start_x;
    double center_y = start_y + radius;
    double center_z = start_z;
    
    ROS_INFO("Circle trajectory parameters:");
    ROS_INFO("- Center: (%.3f, %.3f, %.3f)", center_x, center_y, center_z);
    ROS_INFO("- Radius: %.3f m", radius);
    ROS_INFO("- Angular velocity: %.6f rad/s", angular_velocity);
    ROS_INFO("- Period: %.1f seconds", period);
    ROS_INFO("- Linear speed: %.3f m/s", speed);
    
    // 生成轨迹点
    trajectory.clear();
    double current_time = 0;
    
    // 生成2圈圆形轨迹
    int num_circles = 2;  // 固定跑2圈
    int total_points = (int)(period * num_circles / sample_time);
    
    for (int i = 0; i <= total_points; i++) {
        double t = i * sample_time;
        double angle = angular_velocity * t;
        
        // 圆形参数方程（起始点在圆的最下方）
        double x = center_x + radius * sin(angle);           // X坐标
        double y = center_y - radius * cos(angle);           // Y坐标  
        double z = center_z;                                 // Z坐标（保持不变）
        
        // 计算圆形运动速度（切线方向）
        double vx = -radius * angular_velocity * sin(angle);  // X方向速度
        double vy = radius * angular_velocity * cos(angle);   // Y方向速度
        double vz = 0.0;                                     // Z方向速度保持0
        
        // 创建轨迹点 (16列格式)
        std::vector<double> traj_point(16, 0.0);
        traj_point[0] = x;      // x
        traj_point[1] = y;      // y  
        traj_point[2] = z;      // z
        traj_point[3] = 0.0;        // phi (roll)
        traj_point[4] = 0.0;        // theta (pitch)
        traj_point[5] = 0.0;        // psi (yaw) - 保持固定朝向
        traj_point[6] = vx;         // u (x velocity)
        traj_point[7] = vy;         // v (y velocity)
        traj_point[8] = vz;         // w (z velocity)
        traj_point[9] = 0.0;    // p (roll rate)
        traj_point[10] = 0.0;   // q (pitch rate)
        traj_point[11] = 0.0;   // r (yaw rate)
        traj_point[12] = 0.0;   // u1 (control input)
        traj_point[13] = 0.0;   // u2
        traj_point[14] = 0.0;   // u3
        traj_point[15] = 0.0;   // u4
        
        trajectory.push_back(traj_point);
        current_time += sample_time;
    }
    
    // 添加额外的点用于MPC预测视野
    if (!trajectory.empty()) {
        std::vector<double> last_point = trajectory.back();
        // 将最后一点的速度设为0（停止状态）
        last_point[6] = 0.0; // vx
        last_point[7] = 0.0; // vy
        last_point[8] = 0.0; // vz
        
        for (int i = 0; i < 20; i++) {
            trajectory.push_back(last_point);
        }
    }
    
    number_of_steps = trajectory.size();
    
    ROS_INFO("Generated dynamic circle trajectory:");
    ROS_INFO("- Start: (%.3f, %.3f, %.3f)", start_x, start_y, start_z);
    ROS_INFO("- Center: (%.3f, %.3f, %.3f)", center_x, center_y, center_z);
    ROS_INFO("- Radius: %.3f m", radius);
    ROS_INFO("- Speed: %.3f m/s", circle_speed);
    ROS_INFO("- Period per circle: %.1f seconds", period);
    ROS_INFO("- Number of circles: %d", num_circles);
    ROS_INFO("- Total duration: %.1f seconds", period * num_circles);
    ROS_INFO("- Number of points: %d", number_of_steps);
}
