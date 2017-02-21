function score = compute_p(cx, cy, x, y, sig_x2, sig_y2, sig_xy)
%(cx, cy) = tracker center
%(x, y) = event location

dx = x - cx;
dy = y - cy;

%compute the determinant of the covariance matrix
det = sig_x2*sig_y2 - sig_xy*sig_xy;

%use the determinant for computing its inverse
tmp = (1/det)*(dx*dx*sig_y2 - 2*dx*dy*sig_xy + dy*dy*sig_x2);

%compute the resulting probability
score = 1/(2*pi*sqrt(det))*exp(-0.5*tmp);
