addpath('~/Research/gpml-matlab-v3.4-2013-11-11/');
startup()
%%
close all;

write_fig = 0;

%meanfunc = {@meanSum, {@meanLinear, @meanConst}}; 
%hyp.mean = [0.5; 1];

meanfunc = @meanZero;
hyp.mean = [];

covfunc = @covSEiso; 
ell = 1; 
sf = 0.5; 
hyp.cov = log([ell; sf]);

likfunc = @likGauss; 
sn = 0.1; 
hyp.lik = log(sn);


%Generate the data

x = -2 + 4*rand(20,1);
y = exp(cos(x)).^2 + 0.05*randn(size(x));

%n = 20;
%x = gpml_randn(0.3, n, 1);
%K = feval(covfunc{:}, hyp.cov, x);
%mu = feval(meanfunc{:}, hyp.mean, x);
%y = chol(K)'*gpml_randn(0.15, n, 1) + mu + exp(hyp.lik)*gpml_randn(0.2, n, 1);
 

%nlml = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y)
z = linspace(-2, 4, 101)';
ytrue = exp(cos(z)).^2;
[m s2] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, x, y, z);

figure(2)
set(gca, 'FontSize', 24)
f = [m+2*sqrt(s2); flipdim(m-2*sqrt(s2),1)];
fill([z; flipdim(z,1)], f, [7 7 7]/8);

hold on; 
plot(z, m, 'LineWidth', 2); 
plot(z, ytrue,'k' ,'LineWidth', 2); 

plot(x, y, '+', 'MarkerSize', 12)
grid on
xlabel('input, x')
ylabel('output, y')

%% Learn 2D surface
[X1,X2] = meshgrid(-pi:pi/16:+pi, -pi:pi/16:+pi);

Y = sin(X1).*sin(X2) + 0.1*randn(size(X1));

surf(Y); drawnow;

x = [X1(:) X2(:)];
y = Y(:);

covfunc = @covSEiso; 
likfunc = @likGauss; sn = 0.1; hyp.lik = log(sn);
hyp2.cov = [0 ; 0];    
hyp2.lik = log(0.1);

hyp2 = minimize(hyp2, @gp, -20, @infExact, [], covfunc, likfunc, x, y);
%exp(hyp2.lik)
%nlml2 = gp(hyp2, @infExact, [], covfunc, likfunc, x, y)

[m s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x, y, x);

m = reshape(m, size(Y));

figure(2); surf(m);

%% Learn some crazy function! Brap
addpath('../helpers');

%5-D Training Vectors
x = rand(5, 10000);

%Crazy function Matt made up
t_train = (exp(x(1,:).*x(2,:)) - log(x(3,:))).*x(4,:).^5./(1+cos(x(5,:))) + 0.05*randn(1, size(x, 2));

%Test points
x_test = rand(5, 20);
y_test_true = (exp(x_test(1,:).*x_test(2,:)) - log(x_test(3,:))).*x_test(4,:).^5./(1+cos(x_test(5,:)));

% %Learn the GP!

covfunc = @covSEiso; 
likfunc = @likGauss; 
sn = 0.1; 
hyp.lik = log(sn);
hyp2.cov = [0;0];    
hyp2.lik = log(0.1);
hyp2 = minimize(hyp2, @gp, -10, @infExact, [], covfunc, likfunc, x', t_train');

[y_test_pred s2] = gp(hyp2, @infExact, [], covfunc, likfunc, x', t_train', x_test');

% 
% %Calculate mean Euclidian error
 meanError = mean(abs(y_test_pred - y_test_true'));
 fprintf('Mean error: %f \n',  meanError);




