%bump function
%Eq. 10 of olfati-saber[2006] article
%Input z,h, outpuz ph(z)


function [phzout]=phz(z,h)
if h<=0
    disp('invalid h')
    return
end
if h>=1
    disp('invalid h')
    return
end
    
 
    if z>=0
        if z<=1
            if z<h
                phzout = 1;
            end
        end
    end
    if z>=h
        if z<=1
            phzout = (1/2)*(1 + cos(pi*((z-h)/(1-h))));
        end
    end
    if z>1
        phzout=0;
    end
    if z<0
        phzout=0;
    end  
    
end


