function [anguloOUT] = AjustaAngulo(VECTORanguloIN)


for i=1:length(VECTORanguloIN)
    anguloIN = VECTORanguloIN(i);
    if anguloIN>pi
        while anguloIN>pi
            anguloIN = anguloIN-2*pi;
        end
        anguloOUT(i) = anguloIN;
    elseif anguloIN<-pi
        while anguloIN<-pi
            anguloIN = anguloIN+2*pi;
        end
        anguloOUT(i) = anguloIN;
    else
        anguloOUT(i) = anguloIN;
    end
end

anguloOUT=anguloOUT';

