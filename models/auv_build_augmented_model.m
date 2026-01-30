function [A_aug, B_aug, C_aug, Bw] = auv_build_augmented_model(A, B, C)
% auv_build_augmented_model
% Chapter 8 augmentation for constant input disturbance:
%
%   x_{k+1} = A x_k + B u_k + Bw d_k
%   d_{k+1} = d_k
%
% Here d is a surge FORCE (N), entering through the surge channel.
% Therefore Bw is -first column of B (discrete-time), matching plant sign.

nx = size(A,1);

Bw = -B(:,1);

A_aug = [A, Bw;
         zeros(1,nx), 1];

B_aug = [B;
         zeros(1,size(B,2))];

C_aug = [C, zeros(size(C,1),1)];
end
