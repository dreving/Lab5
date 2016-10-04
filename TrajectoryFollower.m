classdef TrajectoryFollower
    
    properties(Constant)
        kxp = 1.5;
        kyp = 1.5;
        ktp = 3.0;
        UpdatePause = .05;
    end
    
    properties(Access = public)
        robotState;
        signal1;
        signal2;
        signal3;
        signal4;
        signal5;
        signal6;
        signal7;
        signal8;
        signal9;
        signal10;
        signal11;
    end

    properties(Access = private)
    
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = public)     
        
        function [u_V, u_w] = feedback(obj,robPose,goalToWorld,dt)
             %TODO, Figure out how input parameters goalToWorld, actual to
             %World, dt, and previous error
             r_r_p = robPose.aToB()*(goalToWorld.getPoseVec() - robPose.getPoseVec());
             up = [obj.kxp 0 0; 0 obj.kyp obj.ktp]*r_r_p;
             u = up;
             u_V = u(1);
             u_w = u(2);             
        end
        
        function obj = TrajectoryFollower(robot, robotTrajectoryModel)
            %initialization           
            enableFeedback = 1;
            t_f = robotTrajectoryModel.t_f + 1; %add extra second
            n = floor(t_f/TrajectoryFollower.UpdatePause)+1;
            obj.robotState = RobotState(n);
            %get reference to reference and actual state
            actual_robot = obj.robotState;
            t = actual_robot.t;
            w = actual_robot.w;
            V = actual_robot.V;
            s = actual_robot.s;
            x = actual_robot.x;
            y = actual_robot.y;
            th = actual_robot.th;
            figure('units', 'normalized', 'outerposition', [0 0 1 1]);
            hold on;
            %initilize figures and signals
            x_g_ref = zeros(1,n);
            y_g_ref = zeros(1,n);
            th_g_ref = zeros(1,n);
            obj.signal1 = plot(t, x_g_ref, 'm-^', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            obj.signal2 = plot(t, y_g_ref, 'm-p', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            obj.signal3 = plot(t, th_g_ref, 'm-o', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            obj.signal4 = plot(t, x, 'c-^', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            obj.signal5 = plot(t, y, 'c-p', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            obj.signal6 = plot(t, th, 'c-o', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            axis auto;
            xlabel('Time');
            ylabel('X Y TH');
            legend('x_r_e_f', 'y_r_e_f', 'th_r_e_f', 'x_a_c_t', 'y_a_c_t', 'th_a_c_t');
            title(['Reference (magenta) & Actual (cyan) Trajectory']);        
                                    
            figure('units', 'normalized', 'outerposition', [0 0 1 1]);
            hold on;
            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            title(['Reference (magenta circles) & Actual (cyan line) Trajectory (x vs. y) in world coord.']);
            obj.signal7 = plot(x_g_ref, y_g_ref, 'm-o', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            obj.signal8 = plot(x, y, 'c-', 'Linewidth', 1);
            hold on;
            xlabel('X');
            ylabel('Y');
            legend('ref', 'act');
            
            figure('units', 'normalized', 'outerposition', [0 0 1 1]);
            hold on;
            axis auto;
            title(['Error in body coord']);
            err_x_g_ref = zeros(1,n);
            err_y_g_ref = zeros(1,n);
            err_th_g_ref = zeros(1,n);
            obj.signal9 = plot(t, err_x_g_ref, 'r-^', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            obj.signal10 = plot(t, err_y_g_ref, 'r-p', 'Linewidth', 1, 'MarkerSize', 10);
            hold on;
            obj.signal11 = plot(t, err_th_g_ref, 'r-o', 'Linewidth', 1, 'MarkerSize', 10);
            xlabel('Time');
            ylabel('X Y TH');
            legend('x_e_r_r', 'y_e_r_r', 'th_e_r_r');

            t_i = 0;
            dt_i = 0;
            prevX = getX;
            prevY = getY;
            pT = getT;
            cT = pT;
            curX = prevX;
            curY = prevY;
            i = actual_robot.i;
            firstIteration = 0;
            while(t_i <= t_f)
                if(firstIteration == 0)
                    while(eq(cT, pT))     
                        cT = getT;
                        curX = getX;
                        curY = getY;
                        pause(.001);
                    end
                    dt_i = cT - pT; 
                    pT = cT;
                    set(obj.signal1, 'xdata', [get(obj.signal1,'xdata') t(i)], 'ydata', [get(obj.signal1,'ydata') x_g_ref(i)]);
                    set(obj.signal2, 'xdata', [get(obj.signal2,'xdata') t(i)], 'ydata', [get(obj.signal2,'ydata') y_g_ref(i)]);
                    set(obj.signal3, 'xdata', [get(obj.signal3,'xdata') t(i)], 'ydata', [get(obj.signal3,'ydata') th_g_ref(i)]);
                    set(obj.signal4, 'xdata', [get(obj.signal4,'xdata') t(i)], 'ydata', [get(obj.signal4,'ydata') x(i)]);
                    set(obj.signal5, 'xdata', [get(obj.signal5,'xdata') t(i)], 'ydata', [get(obj.signal5,'ydata') y(i)]);
                    set(obj.signal6, 'xdata', [get(obj.signal6,'xdata') t(i)], 'ydata', [get(obj.signal6,'ydata') th(i)]) 
                    set(obj.signal7, 'xdata', [get(obj.signal7,'xdata') -y_g_ref(i)], 'ydata', [get(obj.signal7,'ydata') x_g_ref(i)]);
                    set(obj.signal8, 'xdata', [get(obj.signal8,'xdata') -y(i)], 'ydata', [get(obj.signal8,'ydata') x(i)]);
                    set(obj.signal9, 'xdata', [get(obj.signal9,'xdata') t(i)], 'ydata', [get(obj.signal9,'ydata') err_x_g_ref(i)]);
                    set(obj.signal10, 'xdata', [get(obj.signal10,'xdata') t(i)], 'ydata', [get(obj.signal10,'ydata') err_y_g_ref(i)]);
                    set(obj.signal11, 'xdata', [get(obj.signal11,'xdata') t(i)], 'ydata', [get(obj.signal11,'ydata') err_th_g_ref(i)]);
 
                    obj.robotState.iPlusPlus;
                    firstIteration = 1;
                end
               
                % 1. UPDATE TIME
                t_i = t_i + dt_i;
                i = actual_robot.i;
                
                % 2. WAIT FOR ENCODER CHANGE
                while(eq(cT, pT))     
                    cT = getT;
                    curX = getX;
                    curY = getY;
                    pause(.001);
                end
                
                % 3. UPDATE STATE (DEAD RECKONING)
                dt_i = cT - pT;  
                vl_i = (curX-prevX)/dt_i;
                vr_i = (curY-prevY)/dt_i;
                pT = cT;
                prevX = curX;
                prevY = curY;

                [V_i , w_i] = RobotModelAdv.vlvrToVw(vl_i, vr_i);
                p_prev = Pose(x(i-1), y(i-1), th(i-1));
                p_i_act = RobotModelAdv.integrateDiffEq(V_i, w_i, dt_i, p_prev);
                t(i) = t_i;
                V(i) = V_i;
                w(i) = w_i;
                s(i) = s(i-1) + (V_i*dt_i);
                x(i) = p_i_act.x;
                y(i) = p_i_act.y;
                th(i) = p_i_act.th;
                
                % 4. UPDATE CONTROL
                p_i_ref = robotTrajectoryModel.getPoseAtTime(t_i);
       
                %get velocity from open loop 
                [u_ref_V, u_ref_w] = robotTrajectoryModel.getVelocitiesAtTime(t_i);
                [u_p_V, u_p_w] = obj.feedback(p_i_act,p_i_ref,dt_i);
                V_i = u_ref_V + (enableFeedback*u_p_V);
                w_i = u_ref_w + (enableFeedback*u_p_w);
                
                [v_l_U , v_r_U] = RobotModelAdv.VwTovlvr(V_i, w_i);
                [v_l_U , v_r_U] = RobotModelAdv.limitWheelVelocities([v_l_U , v_r_U]);
                %5. SEND CONTROL TO ROBOT
                robot.sendVelocity(v_l_U, v_r_U); 
            
                %6. UPDATE GRAPHS
                x_g_ref(i) = p_i_ref.x;
                y_g_ref(i) = p_i_ref.y;
                th_g_ref(i) = p_i_ref.th; 
                set(obj.signal1, 'xdata', [get(obj.signal1,'xdata') t(i)], 'ydata', [get(obj.signal1,'ydata') x_g_ref(i)]);
                set(obj.signal2, 'xdata', [get(obj.signal2,'xdata') t(i)], 'ydata', [get(obj.signal2,'ydata') y_g_ref(i)]);
                set(obj.signal3, 'xdata', [get(obj.signal3,'xdata') t(i)], 'ydata', [get(obj.signal3,'ydata') th_g_ref(i)]);
                set(obj.signal4, 'xdata', [get(obj.signal4,'xdata') t(i)], 'ydata', [get(obj.signal4,'ydata') x(i)]);
                set(obj.signal5, 'xdata', [get(obj.signal5,'xdata') t(i)], 'ydata', [get(obj.signal5,'ydata') y(i)]);
                set(obj.signal6, 'xdata', [get(obj.signal6,'xdata') t(i)], 'ydata', [get(obj.signal6,'ydata') th(i)]) 
                set(obj.signal7, 'xdata', [get(obj.signal7,'xdata') -y_g_ref(i)], 'ydata', [get(obj.signal7,'ydata') x_g_ref(i)]);
                set(obj.signal8, 'xdata', [get(obj.signal8,'xdata') -y(i)], 'ydata', [get(obj.signal8,'ydata') x(i)]);
                %computer error in body coord.
                r_r_p = p_i_act.aToB()*(p_i_ref.getPoseVec() - p_i_act.getPoseVec());
                err_x_g_ref(i) = r_r_p(1);
                err_y_g_ref(i) = r_r_p(2);
                err_th_g_ref(i) = r_r_p(3);
                set(obj.signal9, 'xdata', [get(obj.signal9,'xdata') t(i)], 'ydata', [get(obj.signal9,'ydata') err_x_g_ref(i)]);
                set(obj.signal10, 'xdata', [get(obj.signal10,'xdata') t(i)], 'ydata', [get(obj.signal10,'ydata') err_y_g_ref(i)]);
                set(obj.signal11, 'xdata', [get(obj.signal11,'xdata') t(i)], 'ydata', [get(obj.signal11,'ydata') err_th_g_ref(i)]);
     
                %7. update logger index (update sim if sim?)
                actual_robot.iPlusPlus;
                
                %8. DELAY MAC CLOCK
                pause(TrajectoryFollower.UpdatePause);
            end
            robot.stop();
        end
    end

    methods(Access = private)

    end
    
    
end

