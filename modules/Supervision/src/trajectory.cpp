#include <fstream>

#include "regexpatterns.h"
#include "logging.h"
#include "trajectory.h"

const std::regex Trajectory::fileHeaderPattern("TRAJECTORY;(" + RegexPatterns::intPattern + ");("
        + RegexPatterns::namePattern + ");" + RegexPatterns::versionPattern + ";("
        + RegexPatterns::intPattern + ");");
const std::regex Trajectory::fileLinePattern("LINE;(" + RegexPatterns::floatPattern + ");("
        + RegexPatterns::floatPattern + ");(" + RegexPatterns::floatPattern + ");("
        + RegexPatterns::floatPattern + ")?;(" + RegexPatterns::floatPattern + ");("
        + RegexPatterns::floatPattern + ")?;(" + RegexPatterns::floatPattern + ")?;("
        + RegexPatterns::floatPattern + ")?;(" + RegexPatterns::floatPattern + ")?;("
        + RegexPatterns::floatPattern + ");(" + RegexPatterns::intPattern + ");ENDLINE;");
const std::regex Trajectory::fileFooterPattern("ENDTRAJECTORY;");

Trajectory::Trajectory(const Trajectory& other) {
    this->id = other.id;
    this->ip = other.ip;
    this->name = other.name;
    this->version = other.version;
    this->points = std::vector<TrajectoryPoint>(other.points);
}

void Trajectory::initializeFromFile(const std::string fileName) {

    using namespace std;
    char trajDirPath[PATH_MAX];
    string errMsg;
    smatch match;
    ifstream file;
    bool isHeaderParsedSuccessfully = false;
    unsigned long nPoints = 0;
    const string ipAddr(fileName);

    UtilGetTrajDirectoryPath(trajDirPath, sizeof (trajDirPath));
    string trajFilePath(trajDirPath);
    trajFilePath += fileName;

    file.open(trajFilePath);
    if (file.is_open()) {
        string line;
        errMsg = "Encountered unexpected end of file while reading file <" + trajFilePath + ">";
        for (unsigned long lineCount = 0; getline(file, line); lineCount++) {
            if (lineCount == 0) {
                if (regex_search(line, match, this->fileHeaderPattern)) {
                    this->id = stoi(match[1]);
                    this->name = match[2];
                    this->version = 0;
                    nPoints = stoul(match[6]);
                    this->points.reserve(nPoints);
                    isHeaderParsedSuccessfully = true;
                }
                else {
                    errMsg = "The header of trajectory file <" + trajFilePath + "> is badly formatted";
                    break;
                }
            }
            else if (lineCount > 0 && !isHeaderParsedSuccessfully) {
                errMsg = "Attempt to parse trajectory file <" + trajFilePath + "> before encountering header";
                break;
            }
            else if (lineCount > nPoints + 1) {
                errMsg = "Trajectory line count of file <" + trajFilePath
                        + "> does not match specified line count";
                break;
            }
            else if (lineCount == nPoints + 1) {
                if (regex_search(line, match, fileFooterPattern)) {
                    file.close();
                    inet_pton(AF_INET, ipAddr.c_str(), &this->ip);
                    LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", trajFilePath.c_str());
                    return;
                }
                else {
                    errMsg = "Final line of geofence file <" + trajFilePath + "> badly formatted";
                    break;
                }
            }
            else {
                if (regex_search(line, match, fileLinePattern)) {
                    TrajectoryPoint point;
                    point.setTime(stod(match[1]));
                    point.setXCoord(stod(match[2]));
                    point.setYCoord(stod(match[3]));
                    if (match[4].matched)
                        point.setZCoord(stod(match[4]));
                    point.setHeading(stod(match[5]));
                    if (match[6].matched)
                        point.setLongitudinalVelocity(stod(match[6]));
                    if (match[7].matched)
                        point.setLateralVelocity(stod(match[7]));
                    if (match[8].matched)
                        point.setLongitudinalAcceleration(stod(match[8]));
                    if (match[9].matched)
                        point.setLateralAcceleration(stod(match[9]));
                    point.setCurvature(stod(match[10]));
                    point.setMode(static_cast<Trajectory::TrajectoryPoint::ModeType>(stoi(match[11])));
                    points.push_back(point);
                }
                else {
                    errMsg = "Line " + to_string(lineCount) + " of trajectory file <"
                            + trajFilePath + "> badly formatted";
                    break;
                }
            }
        }
        file.close();
        LogMessage(LOG_LEVEL_DEBUG, "Closed <%s>", trajFilePath.c_str());
        LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
        throw invalid_argument(errMsg);
    }
    else {
        errMsg = "Unable to open file <" + trajFilePath + ">";
        LogMessage(LOG_LEVEL_ERROR, errMsg.c_str());
        throw ifstream::failure(errMsg);
    }
}


CartesianPosition Trajectory::TrajectoryPoint::getCartesianPosition() {
    CartesianPosition retval;
    retval.xCoord_m = this->getXCoord();
    retval.yCoord_m = this->getYCoord();
    try {
        retval.zCoord_m = this->getZCoord();
    } catch (std::out_of_range e) {
        LogMessage(LOG_LEVEL_WARNING, "Casting trajectory point to cartesian position: optional z value assumed to be 0");
        retval.zCoord_m = 0.0;
    }
    retval.heading_deg = this->getHeading() * 180.0 / M_PI;
    return retval;
}
