function [Track_F,Track_R,WheelBase_L,WheelBase_R]=compute_wb(Points,i)

Track_F=Points{11,2}(i)-Points{23,2}(i);
Track_R=Points{35,2}(i)-Points{47,2}(i);

WheelBase_L=Points{11,1}(i)-Points{35,1}(i);
WheelBase_R=Points{23,1}(i)-Points{47,1}(i);