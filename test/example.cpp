//
// Created by aass on 30/01/19.
//


#include <string>
#include <iostream>

#include <fstream>
#include <cmath>
#include <cstdio>
#include <stdlib.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include "VoriGraph.h"
#include "TopoGraph.h"
#include "cgal/CgalVoronoi.h"
#include "cgal/AlphaShape.h"
#include "qt/QImageVoronoi.h"

#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/filesystem/path.hpp>

#include <QApplication>

#include <QMessageBox>

#include "RoomDect.h"

#include "roomGraph.h"
#include "Denoise.h"
#include <sys/stat.h>
#include <sys/types.h>

#include "cgal/AlphaShapeRemoval.h"

using namespace std;

#include <sstream>

template<typename T>
std::string NumberToString(T Number) {
    std::ostringstream ss;
    ss << Number;
    return ss.str();
}

int nearint(double a) {
    return ceil( a ) - a < 0.5 ? ceil( a ) : floor( a );
}

VoriConfig *sConfig;

int main(int argc, char *argv[]) {
    if (argc < 2) {
        cout << "Usage " << argv[0]
             << " RGBimage.png <resolution door_wide corridor_wide noise_precentage(0-100) record_time(0 or 1)>"
             << endl;
        return 255;
    }
    double door_wide = 1.15, corridor_wide = 2, res = 0.05;
    double noise_percent = 1.5;
    bool record_time = false;
    if (argc > 2) {
        res = atof( argv[2] );
        if (argc > 4) {
            door_wide = atof( argv[3] ) == -1 ? 1.15 : atof( argv[3] );
            corridor_wide = atof( argv[4] ) == -1 ? 1.35 : atof( argv[4] );

            if (argc > 5) {
                noise_percent = atof( argv[5] );
                if (argc > 6)
                    record_time = true;
            }
        }
    }

    sConfig = new VoriConfig();
    sConfig->doubleConfigVars["alphaShapeRemovalSquaredSize"] = 625;
    sConfig->doubleConfigVars["firstDeadEndRemovalDistance"] = 100000;
    sConfig->doubleConfigVars["secondDeadEndRemovalDistance"] = -100000;
    sConfig->doubleConfigVars["thirdDeadEndRemovalDistance"] = 0.25 / res;
    sConfig->doubleConfigVars["fourthDeadEndRemovalDistance"] = 8;
    sConfig->doubleConfigVars["topoGraphAngleCalcEndDistance"] = 10;
    sConfig->doubleConfigVars["topoGraphAngleCalcStartDistance"] = 3;
    sConfig->doubleConfigVars["topoGraphAngleCalcStepSize"] = 0.1;
    sConfig->doubleConfigVars["topoGraphDistanceToJoinVertices"] = 10;
    sConfig->doubleConfigVars["topoGraphMarkAsFeatureEdgeLength"] = 16;
    sConfig->doubleConfigVars["voronoiMinimumDistanceToObstacle"] = 0.25 / res;
    sConfig->doubleConfigVars["topoGraphDistanceToJoinVertices"] = 4;


    clock_t start;
    start = clock();

    int black_threshold = 210;
    bool de = DenoiseImg( argv[1], "clean.png", black_threshold, 18, noise_percent );
    if (de)
        cout << "Denoise run successed!!" << endl;

    clock_t afterDenoise = clock();
    QImage test;
    test.load( "clean.png" );

    bool isTriple;
    analyseImage( test, isTriple );

    double AlphaShapeSquaredDist =
            (sConfig->voronoiMinimumDistanceToObstacle()) * (sConfig->voronoiMinimumDistanceToObstacle());
    performAlphaRemoval( test, AlphaShapeSquaredDist, MAX_PLEN_REMOVAL );
    test.save( "afterAlphaRemoval.png" );

    clock_t afterAlphaRemoval = clock();
    std::vector<topo_geometry::point> sites;
    bool ret = getSites( test, sites );

    int remove_alpha_value = 3600;
    double a = door_wide < corridor_wide ? door_wide + 0.1 : corridor_wide - 0.1;
    int alpha_value = ceil( a * a * 0.25 / (res * res));
    sConfig->doubleConfigVars["alphaShapeRemovalSquaredSize"] = alpha_value;
    std::cout << "a = " << a << ", where alpha = " << alpha_value << std::endl;
    clock_t loop_start = clock();
    // create the voronoi graph and vori graph
    VoriGraph voriGraph;
    ret = createVoriGraph( sites, voriGraph, sConfig );
    printGraphStatistics( voriGraph );

    clock_t generatedVG = clock();
    QImage alpha = test;
    AlphaShapePolygon alphaSP, tem_alphaSP;
    AlphaShapePolygon::Polygon_2 *poly = alphaSP.performAlpha_biggestArea( alpha, remove_alpha_value, true );
    if (poly) {
        cout << "Removing vertices outside of polygon" << endl;
        removeOutsidePolygon( voriGraph, *poly );
    }
    AlphaShapePolygon::Polygon_2 *tem_poly = tem_alphaSP.performAlpha_biggestArea( alpha,
                                                                                   sConfig->alphaShapeRemovalSquaredSize(),
                                                                                   false );
    voriGraph.joinHalfEdges_jiawei();
    cout << "size of Polygons: " << tem_alphaSP.sizeOfPolygons() << endl;

    clock_t rmOut = clock();
    std::list<std::list<VoriGraphHalfEdge>::iterator> zeroHalfEdge;
    for (std::list<VoriGraphHalfEdge>::iterator pathEdgeItr = voriGraph.halfEdges.begin();
         pathEdgeItr != voriGraph.halfEdges.end(); pathEdgeItr++) {
        if (pathEdgeItr->distance <= EPSINON) {
            zeroHalfEdge.push_back( pathEdgeItr );
        }
    }
    for (std::list<std::list<VoriGraphHalfEdge>::iterator>::iterator zeroHalfEdgeItr = zeroHalfEdge.begin();
         zeroHalfEdgeItr != zeroHalfEdge.end(); zeroHalfEdgeItr++) {
        voriGraph.removeHalfEdge_jiawei( *zeroHalfEdgeItr );
    }

    clock_t zeroHf = clock();
    if (sConfig->firstDeadEndRemovalDistance() > 0.) {
        voriGraph.markDeadEnds();
        removeDeadEnds_addFacetoPolygon( voriGraph,
                                         sConfig->firstDeadEndRemovalDistance());
        voriGraph.joinHalfEdges_jiawei();
    }
    if (sConfig->secondDeadEndRemovalDistance() > 0.) {
        voriGraph.markDeadEnds();
        removeDeadEnds_addFacetoPolygon( voriGraph, sConfig->secondDeadEndRemovalDistance());
        voriGraph.joinHalfEdges_jiawei();
    }

    gernerateGroupId( voriGraph );
    keepBiggestGroup( voriGraph );

    removeRays( voriGraph );
    voriGraph.joinHalfEdges_jiawei();

    if (sConfig->thirdDeadEndRemovalDistance() > 0.) {
        voriGraph.markDeadEnds();
        removeDeadEnds_addFacetoPolygon( voriGraph, sConfig->thirdDeadEndRemovalDistance());
        voriGraph.joinHalfEdges_jiawei();
//         printGraphStatistics(voriGraph, "Third dead ends");
    }
    if (sConfig->fourthDeadEndRemovalDistance() > 0.) {
        voriGraph.markDeadEnds();
        removeDeadEnds_addFacetoPolygon( voriGraph, sConfig->fourthDeadEndRemovalDistance());
        voriGraph.joinHalfEdges_jiawei();
//         printGraphStatistics(voriGraph, "Fourth dead ends");
    }

    clock_t DeadEndRemoval = clock();

    RoomDect roomtest;
    roomtest.forRoomDect( tem_alphaSP, voriGraph, tem_poly );

    clock_t roomDetect = clock();

    QImage dectRoom = test;
    paintVori_onlyArea( dectRoom, voriGraph );
    string tem_s = NumberToString( nearint( a * 100 )) + ".png";
    dectRoom.save( tem_s.c_str());

    clock_t beforeMerge = clock();

    RMG::AreaGraph RMGraph( voriGraph );
    RMGraph.mergeAreas();
    RMGraph.mergeRoomCell();
    RMGraph.prunning();
    RMGraph.arrangeRoomId();
    RMGraph.show();

    clock_t geneAG = clock();
    double t_wholeloop = geneAG - start;

    RMGraph.mergeRoomPolygons();
    std::cout << "Area Graph generation use time: " << t_wholeloop / CLOCKS_PER_SEC << std::endl;

//    QImage RMGIm = test;
//    RMGraph.draw( RMGIm );
//    RMGIm.save( "roomGraph.png" );
    return 0;
}
