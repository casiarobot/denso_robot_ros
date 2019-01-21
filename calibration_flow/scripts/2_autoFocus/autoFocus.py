import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit, fmin
import yaml

def draw_ROI(img, ROI_size):
    img_center = np.flip(np.array(img.shape[0:2])//2, 0)
    size = np.array(ROI_size)
    ROI_left_up = tuple(img_center - size//2)
    ROI_right_down = tuple(img_center + size//2)
    cv2.rectangle(img, ROI_left_up, ROI_right_down, (255, 0, 0), 2)

def cal_contrast(gray_img, focus_regin_size):
    '''
    Contrast measure based on squared Laplacian
    '''
    def cal_G(img, x, y):
        x_sum = []
        y_sum = []
        for i in range(x-1, x+2, 2):
            x_sum.append(np.abs(img[x, y] - img[i, y]))
        for j in range(y-1, y+2, 2):
            y_sum.append(np.abs(img[x, y] - img[x, j]))
        return np.sum(x_sum) + np.sum(y_sum)

    img = gray_img.astype(int)
    (J, K) = focus_regin_size
    g_sum = np.zeros((J, K))
    for x in range(0, J):
        for y in range(0, K):
            g_sum[x, y] = cal_G(img, x, y)**2
    return np.sum(g_sum)/(J*K)

def img_preprocess(img_origin):
        gray = cv2.cvtColor(img_origin, cv2.COLOR_BGR2GRAY)
        blur = cv2.bilateralFilter(gray, 9, 75, 75)
        # threshold = blur
        return  blur

def draw_img(img, frame_name, winWidth, winheight=None, keep_aspect=True, ROI_size=None, draw_roi=True):
    if winheight is not None:
        winWidth = int(winWidth)
        winheight = int(winheight)
    else:
        ratio = float(img.shape[0]) / float(img.shape[1])
        winWidth = int(winWidth)
        winheight = int(winWidth*ratio)

    if draw_roi and ROI_size is not None:
        draw_ROI(img, ROI_size)

    cv2.namedWindow(frame_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(frame_name, winWidth, winheight)
    cv2.imshow(frame_name, img)
    cv2.waitKey(1000)

def gaus(x, a, mean, sigma):
    return a*np.exp(-(x-mean)**2 / (2*sigma**2))

def gaus_fitting(x, y):
    x = np.asarray(x)
    y = np.asarray(y)
    a0 = max(y)
    mean = sum(x*y)/sum(y)
    sigma = np.sqrt(sum(y * (x - mean)**2) / sum(y))
    popt, pcov = curve_fit(gaus, x, y, p0=[a0, mean, sigma])
    return popt

def find_peak(x, y):
    popt = gaus_fitting(x, y)
    def gaus_minus(x, a, mean, sigma):
        return -gaus(x, a, mean, sigma)
    a, mean, sigma = popt
    x_peak = fmin(gaus_minus, x0=x[len(x)//2], args=(a, mean, sigma))[0]
    peak = (x_peak, gaus(x_peak, *popt))
    return (x, gaus(x, *popt), peak)

def main(DEBUG):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)
        
    BASE = path['AFocus'] if DEBUG else path['ROOT']
    IMAGE_PATH = BASE + 'img/af*.bmp'
    AF_GOAL = BASE + 'goal/af_goal.yaml'
    BF_GOAL = BASE + 'goal/bf_goal.yaml'

    images = sorted(glob.glob(IMAGE_PATH))
    ROI = (35, 35) # heigh, width
    windowWidth = 800
    ls_contrast = []
    for fname in images:
        img0 = cv2.imread(fname)
        img = img_preprocess(img0)
        contrast = cal_contrast(img, ROI)
        ls_contrast.append(contrast)
        print(fname)
        print(contrast)
        draw_img(img, 'image', windowWidth, ROI_size=ROI)
        
    # Guas curve fitting and find peak locaion

    with open(AF_GOAL) as f:
        af_goals = np.array(yaml.load(f))
    wd = af_goals[:, 2]
    x, y_hat, peak = find_peak(wd, ls_contrast)
    print(peak)

    # Save best focal work distance to ros goal
    bestFocus_goals = af_goals[0, :]
    bestFocus_goals[2] = peak[0]
    with open(BF_GOAL, 'w') as f:
        yaml.dump(bestFocus_goals.tolist(), f, default_flow_style=False)

    # plot
    plt.plot(x, ls_contrast, 'b+:', label='data')
    plt.plot(x, y_hat, 'ro:', label='fit')
    plt.legend()
    plt.title('Auto focus')
    plt.xlabel('Len position')
    plt.ylabel('Contrast')
    plt.show()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)