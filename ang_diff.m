function delta = ang_diff(alpha, beta)
   validateattributes(alpha, {'numeric'}, {'vector', 'nonsparse'}, 1);
   if nargin > 1 
      validateattributes(beta, {'numeric'}, {'vector', 'nonsparse', 'size', size(alpha)}, 2);
      alpha = beta - alpha;
   else
      alpha = diff(alpha);
   end
   delta = mod(alpha + pi, 2*pi) - pi;  %constrain to [-pi, pi[. will prefer -pi over +pi
   delta(delta == -pi & alpha >= 0) = pi;         %so force -pi to +pi (only when the original angle was positive)
end