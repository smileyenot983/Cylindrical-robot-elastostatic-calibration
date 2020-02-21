function a = get_A(q, theta, force, param)
% Matrix of displacement dt = A * pi
   J = theta_jac(q, theta, param);     
   % only for joints 
   tmp = zeros(6,param.joint_no);   
   for i = 1:param.joint_no
       tmp(:,i) = J(:,7*i-6);
   end   
   J = tmp;
    
   J = J(1:3,:); % remove angles
   a = J;   
   for i = 1:size(J,2)
       Ji = J(:,i); %put to Ji all elements of column i
       a(:,i) = Ji*Ji'*force(1:3); %write to column of a Ji**2*force
   end
end