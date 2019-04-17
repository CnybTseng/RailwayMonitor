function lh = calwinliklihood(GM,GD,tx,ty,td,radius,sigma)


searchradius = radius;
top = max(ty-searchradius,1);
bottom = min(ty+searchradius,size(GM,1));
left = max(tx-searchradius,1);
right = min(tx+searchradius,size(GM,2));

lh = 0;
for y = top : bottom
	for x = left : right	
		if GD(y,x) > 0
			ed = GD(y,x)-pi/2;
		else
			ed = GD(y,x)+pi/2;
		end
		
		dist = sqrt((double(tx-x))^2+(double(ty-y))^2);
		dlh = GM(y,x)*weight(radius,sigma,dist)*cos(abs(ed-td));
		lh = lh+dlh;
	end
end
