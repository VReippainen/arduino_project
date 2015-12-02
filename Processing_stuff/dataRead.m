function [timedata, data] = dataRead(filename)
	raw_data = csvread(filename);
	%raw_data = raw_data(raw_data(:,3) > 1023);
	timedata = raw_data(:,2);
	data = raw_data(:,3);
end
