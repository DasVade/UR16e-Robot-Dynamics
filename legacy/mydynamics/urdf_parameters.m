function [masses,I_links,com] = urdf_parameters()
% Original supplied URDF inertials, transformed into STANDARD-DH link frames.
% These are model parameters, not experimentally identified ground truth.
persistent data
if isempty(data)
    root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
    data=jsondecode(fileread(fullfile(root,'docs','executed','reconciled_parameters.json')));
end
masses=zeros(6,1);I_links=zeros(3,3,6);com=zeros(3,6);
for k=1:6
    masses(k)=data(k).mass;I_links(:,:,k)=data(k).inertia_DH;com(:,k)=data(k).com_DH_m(:);
end
end
