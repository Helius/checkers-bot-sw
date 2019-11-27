import math 

# Ax + By + C = 0


def calcAngle(xt, yt):
    print("calc fof", xt, yt)
    r1 = 245
    r2 = 176
    
    a = -2*xt
    b = -2*yt
    c = xt*xt + yt*yt + r1*r1 - r2*r2
    print(a, b, c)

    #print("line 0, 200", line(0), line(200))

    ab2 = a*a+b*b
    print("ab2", ab2);

    x0 = -a*c/(a*a+b*b)
    y0 = -b*c/(a*a+b*b)
    print("x0,y0", x0, y0)

    d = 0.0
    d = r1*r1 - (c*c/(a*a+b*b))
    mult = 0.0
    multa = math.sqrt((a*a*d)/ab2)
    multb = math.sqrt((b*b*d)/ab2)
    print("d", math.sqrt(d))
    print("mults", multa, multb)

# we have to invert sign if -a/b>0
    diff = -a/float(b)
    print("diff", -a, b, diff)

    ax = x0 - multb
    bx = x0 + multb

    if diff >= 0:
        ay = y0 - multa
        by = y0 + multa
    else:
        ay = y0 + multa
        by = y0 - multa

    print(ax,ay, bx,by)
    return int(ax),int(ay),int(bx),int(by)


def check(x, y, x0, y0, x1, y1):
    print("check for", x, y)
    value = calcAngle(x, y)
    if value == (x0, y0, x1, y1):
        print ("OK")
    else:
        print("Fuck NO!", value)

check(-200, 100, -234, -72, -83, 230);
check(-200, 100, -234, -72, -83, 230);
check(-200, 100, -234, -72, -83, 230);
check(-200, 100, -234, -72, -83, 230);
check(-200, 100, -234, -72, -83, 230);
check(-200, 100, -234, -72, -83, 230);
check(-200, 100, -234, -72, -83, 230);
check(-200, 100, -234, -72, -83, 230);
check(-200, 100, -234, -72, -83, 230);


#print ("result", ax, ay, bx, by)
#print ("angles", 180*math.atan2(ay,ax)/3.14, 180*math.atan2(by,bx)/3.14)

#print(ax-xt,yt-ay, 180*math.atan2(yt-ay, ax-xt)/3.14)
