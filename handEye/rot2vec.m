function [ phi ] = rot2vec( C )
% vrrotmat2vec converts the rotation matrix C into its axis-angle
% representation. Resulting C = [axis angle], thus phi=[angle * axis]
C = vrrotmat2vec(C); phi = C(1:3)*C(4);
phi = phi';
end


