function I = drawrect(I,x,y,w,h)

for i = y:-1:y-h+1
	I(i,x) = 255;
	I(i,x+w-1) = 255;
end

for j = x:x+w-1
	I(y,j) = 255;
	I(y-h+1,j) = 255;
end