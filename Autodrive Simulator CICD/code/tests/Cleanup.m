% Read original file
fname = 'validate_sim_results.m';
fid = fopen(fname,'r','n','UTF-8');  % open as UTF-8
txt = fread(fid,'*char')';
fclose(fid);

% Keep only ASCII characters
txt = regexprep(txt,'[^\x00-\x7F]','');

% Write cleaned file
fid = fopen('validate_sim_results1.m','w');
fwrite(fid, txt, 'char');
fclose(fid);
