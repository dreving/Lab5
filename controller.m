classdef controller
    properties(Constant)
      kxp = 0;
      kxd = 0;
      kyp = 0;
      kyd = 0;
      ktp = 0;
      ktd = 0;
      maxSteer = pi/2;
      maxV = .5;
    end
    properties(Access = public)
        r_r_prev = [0;0;0];
    end
    
     methods(Static = true)
         function u = feedback(obj)
             %TODO, Figure out how input parameters goalToWorld, actual to
             %World, dt, and previous error
             r_r_p = robPose.aToB()*(goalToWorld.getPoseVec() - actToWorld.getPoseVec());
             up = [obj.kxp 0 0; 0 obj.kyp obj.ktp]*r_r_p;
             deriv_error = (r_r_p - obj.r_r_prev)./dt;
             ud = [obj.kxd 0 0; 0 obj.kyd obj.ktd]*deriv_error;
             u = up+ud;
             obj.r_r_prev = r_r_p; %idk if this works
             
%scaling code for steering to hard. Commented out to be implemented when added to feed forward             
%              if u(2) > controller.maxSteer
%                  u = u .*(controller.maxSteer/u(2));
%              end
         end
    end
     methods(Access = public)
         
     end
     methods(Access = private)
         
    end
end