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

    #ax = x0 - multb
    bx = x0 + multb

    if diff >= 0:
        #ay = y0 - multa
        by = y0 + multa
    else:
        by = y0 - multa

    print(bx,by)

    # now let's calc angles
    ang0 = 180*math.atan2(by, bx)/3.14
    ang1 = 180*math.atan2(yt-by, bx-xt)/3.14
    print('angles is: ', ang0, ang1)

    return int(bx),int(by),int(ang0),int(ang1)


def check(x, y, x1, y1, ang0, ang1):
    print('\n')
    print("check for", x, y, ang0)
    value = calcAngle(x, y)
    if value == (x1, y1, ang0, ang1):
        print ("OK")
    else:
        print("Fuck NO! expected", (x1, y1, ang0, ang1), "got", value)

check(95,   105,   244, 11, 2, 31)
check(100,  230,   222, 103, 24, 46)
check(100,  330,   175, 170, 44, 64)
check(0,    330,   127, 209, 58, 43)  # 2, 58,5
check(-100, 330,   50, 239, 78, 31);  # 2, 78
check(-110, 110,   1, 244, 89, -50);   # 2, 89
check(0,    200,   173, 172, 44, 9); # 2, 45,5
check(-60,  110,   61, 236, 75, -46);  # 2, 76
check(-170, 110,   -51, 238, 102, -47); # 2, 102

