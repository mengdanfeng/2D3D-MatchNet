function output_to_file(output_pc, keypoints, submapID)
    %output pointcloud in binary file
    fileID = fopen(strcat(base_path,pc_output_folder, num2str(submapID),'.bin'),'w');
    
    logical_keypoints=zeros(size(output_pc,2),1);
    logical_keypoints(keypoints,1)=1;
    output_pc(4,:)=logical_keypoints;
    
    fwrite(fileID,output_pc,'double');
    fclose(fileID);
    disp(num2str(submapID));    
end