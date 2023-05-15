"stat info for img"
from pathlib import Path
from matplotlib import pyplot as plt
from matplotlib import use
from PIL import Image, ImageStat

def get_mask(imgfile):
    img = Image.open(imgfile)
    #print("bands", img.getbands())
    mask = img.getchannel('A')
    #mask.show('mask')
    return mask


def histo_img(imgfile, outfile, mask=None, title=None):
    use('Agg')
    Path(outfile).unlink(missing_ok=True)
    img = Image.open(imgfile)
    maske = None
    if 'A' in img.getbands():
        #print('Alfa channel includet'+ str(outfile))
        alfa = img.getchannel('A')
        #alfa.show()
        maske=alfa
    else:
        if mask:
            maske=mask
    grey = img.convert('L')
    #grey.show()
    stat = ImageStat.Stat(grey, mask=maske)
    # print("grey count", stat.count, "extrema", stat.extrema, "mean", stat.mean, "median", stat.median)
    # print("rms", stat.rms, "var", stat.var, "stddev", stat.stddev)
    hist = grey.histogram()
    X=list(range(0,256))
    plt.clf()
    plt.bar(X, hist)
    if title is None:
        title = imgfile.name
    plt.title("Histogram " + title)
    plt.xlabel("Intensity")
    plt.ylabel("Numbers")
    plt.ylim(0,1000)
    plt.figtext(0.7, 0.8, f"Brightness: {int(stat.mean[0])} {int(stat.median[0])}")
    plt.figtext(0.7, 0.75, f"rms: {int(stat.rms[0])}")
    plt.figtext(0.7, 0.7, f"count: {int(stat.count[0])}")
    plt.figtext(0.7, 0.65, f"alpha: {maske is not None}")
    plt.figtext(0.7, 0.6, f"stdev (contrast): {int(stat.stddev[0])}")
    #plt.figtext(0.7, 0.55, f"Brightness: {int(stat.mean[0])}")
    plt.savefig(outfile)
    plt.clf()
    #plt.show()

if __name__ == "__main__":
    testpicture = Path(__file__).parent / 'testpictures/analytic/testtarget1/render0/image0.png'
    testpicture2 = Path(__file__).parent / 'testpictures/analytic/planer/render0/image0.png'
    #testpicture2 = Path(__file__).parent / 'testpictures/analytic/kugler/analog_im_wrap1.png'
    outputfolder = Path(__file__).parent.parent / 'tmp'
    outputfolder.mkdir(exist_ok=True)
    outfile = outputfolder / 'testout.png'
    print("histo_img", testpicture, outfile)
    histo_img(testpicture, outfile)
    histo_img(testpicture2, outputfolder / 'testout2.png')
