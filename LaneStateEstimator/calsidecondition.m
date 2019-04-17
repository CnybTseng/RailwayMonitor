function calsidecondition(camera,model)

nearest = camera.fy*camera.h/(camera.yreso-camera.cy);

minphi1 = model.b/(2*nearest)-camera.cx/camera.fx;
fprintf('lower bound of yaw angle: %f.\n',minphi1);

maxphi1 = -model.b/(2*nearest)+(camera.xreso-camera.cx)/camera.fx;
fprintf('upper bound of yaw angle: %f.\n',maxphi1);