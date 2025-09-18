import os.path
import argparse
import sys
import json
import shutil
import configparser

from pathlib import Path
from datetime import datetime

## Config.ini configuration
# Unit: FPS. Please fill in according to the actual dataset frame rate,
# otherwise it will cause inaccurate matching.
frameRate = -1
# Unit: ms. When the timestamp range difference of a matched frame group
# is greater than or equal to tspRangeThreshold, the file name will be marked.
tspRangeThreshold = -1

## Parsed automatically by Python script, DO NOT modify
primaryId = '-1' # Host ID
mainPythonWorkPath = "" # Working directory of SyncFramesMain.py
sourceFramesPath = ""  # DO NOT modify, fixed value, path to TotalMode directory
sourceRootPath = "" # Root directory containing TotalMode
resultPath = "" # Path to save sync analysis results
streamProfileDict = dict() # Store Python results

class PictureInfo(object):
    class Struct(object):
        def __init__(self, deviceId, sensorType, syncTimeStamp, picturePath, deviceIdFull, fileExt):
            self.deviceId = deviceId
            self.sensorType = sensorType
            self.syncTimeStamp = syncTimeStamp
            self.picturePath = picturePath
            self.deviceIdFull = deviceIdFull
            self.fileExt = fileExt

    def make_struct(self, deviceId, sensorType, syncTimeStamp, picturePath, deviceIdFull, fileExt):
        return self.Struct(deviceId, sensorType, syncTimeStamp, picturePath, deviceIdFull, fileExt)

class StreamProfileInfo:
    def __init__(self, sensorType, width, height, format, fps):
        self.sensorType = sensorType
        self.width = width
        self.height = height
        self.format = format
        self.fps = fps

def initPrimaryId():
    global primaryId

    deviceInfoPath = f"{sourceRootPath}/DevicesInfo.txt"
    if not os.path.exists(deviceInfoPath):
        print(f"Initialize primaryId failed. {deviceInfoPath} not exists")
        return False

    with open(deviceInfoPath, 'r') as f:
        data = json.load(f)

    if not 'devices' in data:
        print(f"Initialize primaryId failed.  {deviceInfoPath} not contain item 'devices'")
        return False

    for item in data['devices']:
        if 'isPrimaryDevice'  in item and 'index' in item and item['isPrimaryDevice']:
            primaryId = str(item['index'])
            return True

    return False

def initSyncConfigParams():
    global frameRate, tspRangeThreshold

    configPath = f"{mainPythonWorkPath}/Config.ini"
    if not os.path.exists(configPath):
        print(f"Initliaze synchronized config parameter failed. {configPath} not exists.")
        return False

    # Create ConfigParser object
    config = configparser.ConfigParser()

    # Read ini file
    config.read(configPath, 'utf-8')

    if not config.has_option('Parameter', 'frameRate'):
        print(f"Initliaze synchronized config parameter failed. Not define 'frameRate' in ${configPath}")
        return False

    if not config.has_option('Parameter', 'tspRangeThreshold'):
        print(f"Initliaze synchronized config parameter failed. Not define 'tspRangeThreshold' in ${configPath}")
        return False

    frameRate = config.getfloat('Parameter', 'frameRate')
    if frameRate <= 0 or frameRate >= 1000:
        print(f"Initliaze synchronized config parameter failed. Invalid frameRate={frameRate}")
        return False

    tspRangeThreshold = config.getfloat('Parameter', 'tspRangeThreshold')
    if tspRangeThreshold <= 0:
        print(f"Initliaze synchronized config parameter failed. Invalid tspRangeThreshold={tspRangeThreshold}")
        return False

    return True

def getDeviceCount():
    deviceInfoPath = f"{sourceRootPath}/DevicesInfo.txt"
    if not os.path.exists(deviceInfoPath):
        print(f"Get device count failed. {deviceInfoPath} not exists")
        return 0

    with open(deviceInfoPath, 'r') as f:
        data = json.load(f)

    if not 'devices' in data:
        print(f"Get device count failed.  {deviceInfoPath} not contain item 'devices'")
        return 0

    return len(data['devices'])

def initProfileInfoDict():
    global streamProfileDict

    streamProfileDict = dict()
    profilePath = f"{sourceRootPath}/StreamProfileInfo.txt"
    if not os.path.exists(profilePath):
        print(f"Initialize profile information dictionary failed. {profilePath} not exists")
        return False

    with open(profilePath, 'r') as f:
        data = json.load(f)

    if not 'streamProfiles' in data:
        print(f"Initialize profile information dictionary failed. {profilePath} not contain item 'streamProfiles'")
        return False

    for item in data['streamProfiles']:
        if 'sensorType' in item and 'width' in item and 'height' in item and 'format' in item and 'fps' in item:
            profileInfo = StreamProfileInfo(item['sensorType'], item['width'], item['height'], item['format'], item['fps'])
            streamProfileDict.setdefault(item['sensorType'], profileInfo)
        else:
            print(f"Initialize profile information dictionary. Error invalid StreamProfile, {item}")

    return len(streamProfileDict.items()) > 0

def initResultDir():
    global resultPath
    timeText = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    resultPath = f"{sourceRootPath}/results-{timeText}"
    if not os.path.exists(resultPath):
        os.makedirs(resultPath)

def isFrameFile(fileName):
    fileExt = os.path.splitext(fileName)
    if len(fileExt) < 2:
        return False

    frameExts = {".jpeg", ".jpg", ".png", ".raw", "bmp"}
    return fileExt[1] in frameExts

def initPictureInfoDictionary(rootFilePath, pictureInfoDict):
    frameFileCount = 0
    for dirpath, dirnames, filenames in os.walk(rootFilePath):
        # print(f"dirpath={dirpath}, dirpath.basename=" + os.path.basename(dirpath))
        for fileName in filenames:
            filePath = os.path.join(dirpath, fileName)
            if not isFrameFile(fileName):
                continue

            frameFileCount += 1
            fileNameNoExt = os.path.splitext(fileName)[0]
            pictureNameList = fileNameNoExt.split('_')
            deviceId = pictureNameList[2][5:]
            sensorType = pictureNameList[0].replace('#', '_')
            syncTimeStamp = int(pictureNameList[5][1:])

            # Create struct
            pictureInfo = PictureInfo()
            info = pictureInfo.make_struct(deviceId, sensorType, syncTimeStamp, filePath, os.path.basename(dirpath), os.path.splitext(fileName)[1])

            # Format dictionary: one key corresponds to one array
            pictureInfoDict.setdefault(deviceId, []).append(info)

    if 0 == frameFileCount:
        print("initialize pictureInfoDict failed. Not found frame file")


def matchFrame(pictureInfoDict):
    global primaryId, frameRate
    global streamProfileDict

    # Get comparison image array, take the Color array of the first device for comparison
    compareList = []
    for key in pictureInfoDict:
        listTmp = list(pictureInfoDict[key])
        for info in listTmp:
            if info.deviceId == primaryId and info.sensorType == 'color':
                compareList.append(info)

    # Dynamically analyze based on stream opening status to reduce loop iterations
    hasColorProfile = 'OB_SENSOR_COLOR' in streamProfileDict
    hasDepthProfile = 'OB_SENSOR_DEPTH' in streamProfileDict
    hasIRProfile = 'OB_SENSOR_IR' in streamProfileDict
    hasIRLeftProfile = 'OB_SENSOR_IR_LEFT' in streamProfileDict
    hasIRRightProfile = 'OB_SENSOR_IR_RIGHT' in streamProfileDict

    # Match adjacent frames
    resultDict = {}
    consumedList = []
    dictIndex = 0
    frameTspHalfGap = int(1000.0/frameRate/2.0+0.5)
    for comparePic in compareList:
        resultDict.setdefault(dictIndex, []).append(comparePic)
        for key in pictureInfoDict:
            listTmp = list(pictureInfoDict[key])
            # Compare Color
            if hasColorProfile:
                for info in listTmp:
                    if info.deviceId == primaryId and info.sensorType == comparePic.sensorType:
                        continue

                    if info in consumedList:
                        continue

                    if info.sensorType == 'color' and abs(info.syncTimeStamp - comparePic.syncTimeStamp) < frameTspHalfGap:
                        resultDict.setdefault(dictIndex, []).append(info)
                        consumedList.append(info)
                        break

            # Compare Depth
            if hasDepthProfile:
                for info in listTmp:
                    if info.deviceId == primaryId and info.sensorType == comparePic.sensorType:
                        continue

                    if info in consumedList:
                        continue

                    if info.sensorType == 'depth' and abs(info.syncTimeStamp - comparePic.syncTimeStamp) < frameTspHalfGap:
                        resultDict.setdefault(dictIndex, []).append(info)
                        consumedList.append(info)
                        break

            # IR
            if hasIRProfile:
                for info in listTmp:
                    if info.deviceId == primaryId and info.sensorType == comparePic.sensorType:
                        continue

                    if info in consumedList:
                        continue

                    if info.sensorType == 'ir' and abs(info.syncTimeStamp - comparePic.syncTimeStamp) < frameTspHalfGap:
                        resultDict.setdefault(dictIndex, []).append(info)
                        consumedList.append(info)
                        break

            # Left IR
            if hasIRLeftProfile:
                for info in listTmp:
                    if info.deviceId == primaryId and info.sensorType == comparePic.sensorType:
                        continue

                    if info in consumedList:
                        continue

                    if info.sensorType == 'ir_left' and abs(info.syncTimeStamp - comparePic.syncTimeStamp) < frameTspHalfGap:
                        resultDict.setdefault(dictIndex, []).append(info)
                        consumedList.append(info)
                        break

            # Right IR
            if hasIRRightProfile:
                for info in listTmp:
                    if info.deviceId == primaryId and info.sensorType == comparePic.sensorType:
                        continue

                    if info in consumedList:
                        continue

                    if info.sensorType == 'ir_right' and abs(info.syncTimeStamp - comparePic.syncTimeStamp) < frameTspHalfGap:
                        resultDict.setdefault(dictIndex, []).append(info)
                        consumedList.append(info)
                        break

        # Calculate deviation
        minTsp = min(resultDict[dictIndex], key=lambda x: x.syncTimeStamp)
        for info in resultDict[dictIndex]:
            info.tspDiff = info.syncTimeStamp - minTsp.syncTimeStamp

        dictIndex += 1

    return resultDict


def safe_make_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)


def handleSyncFrames():
    global resultPath
    global sourceFramesPath
    global streamProfileDict

    fp = open(f"{resultPath}/frameMatchLog.txt", "w")

    if not initProfileInfoDict():
        print("Initialize stream profile dictionary failed. exit")
        return
    print(f"Stream profiles: {streamProfileDict}")
    streamProfileCount = len(streamProfileDict.items())

    pictureInfoDict = {}

    initPictureInfoDictionary(Path(sourceFramesPath), pictureInfoDict)
    if len(pictureInfoDict) <= 0:
        print("init pictureInfoDict failed. pictureInfoDict.len = 0")
        return

    deviceCount = getDeviceCount()
    print("=================device count:%d" % deviceCount)

    resultDict = {}
    resultDict = matchFrame(pictureInfoDict)

    for key in resultDict:
        fp.write("===================result key:%s\n" % key)
        print("===================result key:%s" % key)
        listTmp = list(resultDict[key])
        for info in listTmp:
            fp.write("=====================result value : deviceId:%s\tsensorType:%s\tsyncTimeStamp:%s\tpicturePath:%s\n" % (
                info.deviceId, info.sensorType, info.syncTimeStamp, info.picturePath))
            print("=====================result value : deviceId:%s\tsensorType:%s\tsyncTimeStamp:%s\tpicturePath:%s" % (
                info.deviceId, info.sensorType, info.syncTimeStamp, info.picturePath))

    matchPath = f"{resultPath}/matchFrames/"
    safe_make_dir(matchPath)
    notMatchPath = f"{resultPath}/notMatchFrames/"
    abnormalPath = f"{resultPath}/abnormal/"

    for key in resultDict:
        listTmp = list(resultDict[key])
        if (len(listTmp) == (deviceCount * streamProfileCount)):
            haveAbnormalData = False
            for info in listTmp:
                if info.tspDiff >= tspRangeThreshold:
                    path = matchPath + str(key) + "_" + info.sensorType + "_" + info.deviceIdFull + "_" + str(
                        info.syncTimeStamp) + "_[" + str(info.tspDiff) + "]" + "_xxxxxx" + info.fileExt
                    shutil.copy(info.picturePath, path)
                    haveAbnormalData = True
                else:
                    path = matchPath + str(key) + "_" + info.sensorType + "_" + info.deviceIdFull + "_" + str(
                        info.syncTimeStamp) + "_[" + str(info.tspDiff) + "]" + info.fileExt
                    shutil.copy(info.picturePath, path)

            if haveAbnormalData:
                safe_make_dir(abnormalPath)
                for info in listTmp:
                    if info.tspDiff >= tspRangeThreshold:
                        path = abnormalPath + str(key) + "_" + info.sensorType + "_" + info.deviceIdFull + "_" + str(
                            info.syncTimeStamp) + "_[" + str(info.tspDiff) + "]" + "_xxxxxx" + info.fileExt
                        shutil.copy(info.picturePath, path)
                    else:
                        path = abnormalPath + str(key) + "_" + info.sensorType + "_" + info.deviceIdFull + "_" + str(
                            info.syncTimeStamp) + "_[" + str(info.tspDiff) + "]" + info.fileExt
                        shutil.copy(info.picturePath, path)
        else:
            safe_make_dir(notMatchPath)
            for info in listTmp:
                newFilePath = notMatchPath + str(key) + "_" + info.sensorType + "_" + info.deviceIdFull + "_" + str(
                    info.syncTimeStamp) + "_[" + str(info.tspDiff) + "]" + info.fileExt
                shutil.copy(info.picturePath, newFilePath)

def main():
    if not initPrimaryId():
        print("Initialize primaryId failed. exit.")
        return

    if not initSyncConfigParams():
        print("Initialized Sync config parameter failed. exit.")
        return

    initResultDir()

    print(f"PrimaryId={primaryId}, frameRate={frameRate}, tspRangeThreshold={tspRangeThreshold}")
    handleSyncFrames()

    return


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Synchronize frames support for 'Orbbec Gemini 2'")
    parser.add_argument('frames_dir', type=str, help="Frames directory")
    parser.add_argument('main_script_root_dir', type=str, help="SyncFramesMain.py working directory")
    args = parser.parse_args()

    sourceRootPath = args.frames_dir
    sourceFramesPath = f"{args.frames_dir}/TotalModeFrames"
    mainPythonWorkPath = args.main_script_root_dir
    print(f"Start of Gemini2 synchronize frame. sourceRootPath={sourceRootPath}")

    main()

    print("Finish of Gemini2 synchronize frame.")
