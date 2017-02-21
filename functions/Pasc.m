function P=Pasc(k,n)
if (k>=0)&&(k<=n)
    P=factorial(n)/(factorial(n-k)*factorial(k));
else
    P=0;
end