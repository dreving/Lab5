classdef Controller < handle
    %rename to controller.m 
    properties(Constant)
      kxp = 1.5;%1.5;
      kxd = .02;%.02;
      k_psi_p = 5;%1.5;
      k_psi_d = .02;%.02;
      ktp = 5;
      ktd = .01;
      maxSteer = pi/2;
      maxV = .5;
    end
    properties(Access = public)
        r_r_prev = [0;0;0];
    end
    
     methods(Access = public)
         
         function obj = Controller()
            
         end
         
         function [u_V, u_w] = feedback(obj,robPose,goalToWorld,dt)
             %TODO, Figure out how input parameters goalToWorld, actual to
             %World, dt, and previous error
             r_r_p = robPose.aToB()*(goalToWorld.getPoseVec() - robPose.getPoseVec());
             x_psi_z = [r_r_p(1) atan2(r_r_p(2),r_r_p(1)); r_r_p(3)];
             up = [obj.kxp 0 0; 0 obj.k_psi_p obj.ktp]* x_psi_z;
             deriv_error = ( x_psi_z - obj.r_r_prev)./dt;
             ud = [obj.kxd 0 0; 0 obj.k_psi_d obj.ktd]*deriv_error;
             u = up+ud;
             u_V = u(1);
             u_w = u(2);
             obj.r_r_prev =  x_psi_z; %idk if this works
             
%scaling code for steering to hard. Commented out to be implemented when added to feed forward             
%              if u(2) > controller.maxSteer
%                  u = u .*(controller.maxSteer/u(2));
%              end
         end
    end

     methods(Access = private)
         
    end
end