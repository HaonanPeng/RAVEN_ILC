function [avrn,del_counter,stdd]=postProc(a1)
%This function is to process the data, the strange datas (more than 2std away from mean) 

avr=mean(a1);
stdd=std(a1);
del_counter=0;
for i=1:length(a1)
    if i>length(a1)
        break
    end
    if a1(i)>avr+2*stdd || a1(i)<avr-2*stdd
        a1(i)=[];
        del_counter=del_counter+1;
    end
    
end
avrn=mean(a1);
%stddn=std(a1);



end