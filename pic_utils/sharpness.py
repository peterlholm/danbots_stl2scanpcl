"Calculate sharpness of image"
import cv2 as cv
import numpy as np
from pathlib import Path

def variance_of_laplacian(image):
    """
    compute the Laplacian of the image and then return the focus measure, which is simply the variance of the Laplacian
    """
    return cv.Laplacian(image, cv.CV_64F).var()

def calc_sharpnes(img):
    "return the sharpness as an integer btween 0 and 100?"
    a = variance_of_laplacian(img)
    print (a)
    return a

def gen_sharpness_picture(img):
    "generate a picture showing sharpness areas in image"
    print("xx",img)
    cv.imshow("img", img)
    print("shape", img.shape)
    h,  w = img.shape[:2]
    n = 4
    hp = h//n
    wp = w//n
    print(h,w, hp, wp)
    res = np.empty((n,n))
    nn = 0
    #print(img)
    for x in range(n):
        for y in range(n):
            print ("x", x*wp, "y", y*hp)
            iarr = img[x*wp:wp, y*hp:hp]
            print("sha", iarr.shape)
            print(iarr)
            cv.imshow("uu"+str(nn), iarr )
            cv.waitKey(3000)
            nn += 1
    return False

if __name__ =='__main__':
    picture1 = 'testdata/device/serie1/input/1/color.jpg'
    picture2 = 'testdata/device/serie1/input/1/color.jpg'
    picture1 = 'testdata/wand/Beige_Toothset/piZ2_210907/1/color.jpg'
    picture2 = 'testdata/wand/Beige_Toothset/piZ2_210907/2/color.jpg'

    testpicture = Path(__file__).parent / 'testpictures/analytic/testtarget1/render0/image0.png'
 
    im1 = cv.imread(str(testpicture))
    calc_sharpnes(im1)
    # im2 = cv.imread(picture2)
    # #calc_sharpnes(im2)
    # res = gen_sharpness_picture(im2)
    # #cv.imshow("res", res)
