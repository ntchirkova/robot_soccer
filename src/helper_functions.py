import math

"""
Convenience functions 
"""



def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should
        be in radians) the difference is always based on the closest
        rotation from angle a to angle b.
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def add_angles(angle1, angle2):
    if abs(angle1 + angle2) < math.pi:
        return (angle1 + angle2)
    elif (angle1 + angle2) > math.pi:
        return (angle1 + angle2 - math.pi)
    else:
        return (angle1 + angle2 + math.pi)

def nothing():
    pass

def calibrate_for_lighting(self, base):
    """
    Helper function to calibrate the mask for the lighting situation
    """
    img = cv2.medianBlur(base.copy(),5)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    cv2.namedWindow('image')

    # create trackbars for color change
    cv2.createTrackbar('HL','image',42,255,nothing)
    cv2.createTrackbar('HU','image',84,255,nothing)
    cv2.createTrackbar('SL','image',27,255,nothing)
    cv2.createTrackbar('SU','image',255,255,nothing)
    cv2.createTrackbar('VL','image',41,255,nothing)
    cv2.createTrackbar('VU','image',255,255,nothing)

    while(1):
        rectimg = base.copy()
        # get current positions of four trackbars
        hl = int(cv2.getTrackbarPos('HL','image'))
        hu = int(cv2.getTrackbarPos('HU','image'))
        sl = int(cv2.getTrackbarPos('SL','image'))
        su = int(cv2.getTrackbarPos('SU','image'))
        vl = int(cv2.getTrackbarPos('VL','image'))
        vu = int(cv2.getTrackbarPos('VU','image'))

        lower = np.array([hl, sl, vl])
        upper = np.array([hu, su, vu])

        outimg = cv2.inRange(img.copy(), lower, upper)
        outimg = cv2.erode(outimg, None, iterations=3)
        outimg = cv2.dilate(outimg, None, iterations=2)
        visimg = cv2.cvtColor(outimg, cv2.COLOR_GRAY2RGB)
        vis = np.concatenate((visimg,rectimg), axis=1)


        cv2.imshow('image', vis)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()