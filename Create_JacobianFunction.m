clear all
car = Vehicle_v3();
car.Jacobian_function();
matlabFunction(car.J,'File','CalculateJ_linear');

