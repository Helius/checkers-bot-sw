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
    return int(bx),int(by)


def check(x, y, x1, y1):
    print('\n')
    print("check for", x, y)
    value = calcAngle(x, y)
    if value == (x1, y1):
        print ("OK")
    else:
        print("Fuck NO! expected", (x1, y1), "got", value)

check(95,   105, 244, 11)   # 2
check(100,  230, 222, 103)  # 2
check(100,  330, 175, 170)  # 2
check(0,    330, 127, 209) # 2
check(-100, 330, 50, 239);# 2
check(-110, 110, 1, 244);  # 2
check(0,    200, 173, 172);  # 2
check(-60,  110, 61, 236);  # 2
check(-170, 110, -51, 238);  # 2


#print ("result", ax, ay, bx, by)
#print ("angles", 180*math.atan2(ay,ax)/3.14, 180*math.atan2(by,bx)/3.14)

#print(ax-xt,yt-ay, 180*math.atan2(yt-ay, ax-xt)/3.14)
