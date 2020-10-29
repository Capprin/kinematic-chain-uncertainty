% tests normal assumption for randn

mean = 0;
sd = 1;
num_samples = 1000000;

rng('default') %random generator
samples = sd * randn(num_samples, 1) + mean;

histogram(samples);
title('Normal Distribution, 10^6 Samples');
xlabel('Sample');
ylabel('Frequency');