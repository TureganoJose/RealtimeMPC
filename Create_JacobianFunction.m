clear all
car = Vehicle_v2();
car.Jacobian_function();
matlabFunction(car.J,'File','CalculateJ');