QT += core gui network opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = engine_robotics
TEMPLATE = app

#QMAKE_CXXFLAGS += -m32
CONFIG += c++11
CONFIG += resources_big

DEFINES += ENABLE_STL_SUPPORT

INCLUDEPATH += /usr/include/freetype2/
DEPENDPATH += /usr/include/freetype2/

LIBS += -lfreetype

win32: LIBS += -L$$PWD/freetype/ -lfreetype

INCLUDEPATH += $$PWD/freetype/include
DEPENDPATH += $$PWD/freetype/include

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/freetype/freetype.lib
else:win32-g++: PRE_TARGETDEPS += $$PWD/freetype/libfreetype.a

#"При проведении обыска или осмотра жилья или иного владения лица,
#обыска лица, если привлечение понятых объективно невозможно или
#связано с потенциальной опасностью для их жизни или здоровья,
#соответствующие следственные (розыскные) действия проводятся
#без привлечения понятых. В таком случае ход и результаты проведения обыска ...
#фиксируются доступными техническими средствами путем осуществления непрерывной видеозаписи", - говорится в законе.
#Как писал УНИАН, в Киеве открыто производство против мужчины,
#который звонил на "горячую" линию Фонда гарантирования вкладов физических лиц и оправдывал войну РФ против Украины.

#Linux
linux: {

#Android
 android: {
  LIBS +=  -lGLESv1_CM -lGLESv2
}

#Linux default
 !android: {
   LIBS += -lGL -lGLU -lglut #-lGLEW
   LIBS += -lfreetype
}

}

#Windows
win32: {
   LIBS += -lopengl32 -lglu32  #-lglew32
}

#Windows
win64: {
   LIBS += -lopengl32 -lglu32  #-lglew32
}

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    Engine/IAlgorithm/GJK-EPA/IGjkEpa.cpp \
    Engine/IAlgorithm/IClliping.cpp \
    Engine/IAlgorithm/IQuickClipping.cpp \
    Engine/IAlgorithm/IScanColsingPath.cpp \
    Engine/IAlgorithm/QuickHull/QuickHull.cpp \
    Engine/IAlgorithm/iclliping3d.cpp \
    Engine/ICommon/IColor.cpp \
    Engine/IComponent/IComponent.cpp \
    Engine/IDynamics/IConstraintSolver.cpp \
    Engine/IDynamics/IJoints/IBallAndSocketJoint.cpp \
    Engine/IDynamics/IJoints/IFixedJoint.cpp \
    Engine/IDynamics/IJoints/IHingeJoint.cpp \
    Engine/IDynamics/IJoints/IJoint.cpp \
    Engine/IDynamics/IJoints/ISliderJoint.cpp \
    Engine/IDynamics/IContactSolver.cpp \
    Engine/IDynamics/IDynamicsWorld.cpp \
    Engine/IDynamics/IIntegrateUtil.cpp \
    Engine/IDynamics/IIsland.cpp \
    Engine/IDynamics/IMaterial/IPhysicsMaterial.cpp \
    Engine/IDynamics/IRigidBody.cpp \
    Engine/IDynamics/IStep.cpp \
    Engine/IDynamics/ITimer.cpp \
    Engine/IGeometry/IAABBox3D.cpp \
    Engine/IGeometry/ICollision/IBody/IBody.cpp \
    Engine/IGeometry/ICollision/IBody/ICollisionBody.cpp \
    Engine/IGeometry/ICollision/IBroadPhase/IBroadPhase.cpp \
    Engine/IGeometry/ICollision/IBroadPhase/IDynamicAABBTree.cpp \
    Engine/IGeometry/ICollision/ICollisionContact/IContactGenerator.cpp \
    Engine/IGeometry/ICollision/ICollisionContact/IContactManager.cpp \
    Engine/IGeometry/ICollision/ICollisionContact/IContactManifold.cpp \
    Engine/IGeometry/ICollision/ICollisionContact/IContactManifoldSet.cpp \
    Engine/IGeometry/ICollision/ICollisionContact/IContactPoint.cpp \
    Engine/IGeometry/ICollision/ICollisionWorld.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShape.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShapeBox.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShapeConvex.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShapeHull.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShapeSphere.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/IConcaveMeshShape.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/IConcaveShape.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/IConvexPolyhedronShape.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/IHalfEdgeStructure.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/ITriangleMesh.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/ITriangleShape.cpp \
    Engine/IGeometry/ICollision/ICollisionShapes/ITriangleVertexArray.cpp \
    Engine/IGeometry/ICollision/INarrowPhase/GJK/IGJKIntersection.cpp \
    Engine/IGeometry/ICollision/INarrowPhase/GJK/Simplex.cpp \
    Engine/IGeometry/ICollision/INarrowPhase/ICollisionAlgorithm.cpp \
    Engine/IGeometry/ICollision/INarrowPhase/ICollisionAlgorithmGjkEpa.cpp \
    Engine/IGeometry/ICollision/IOverlappingPair.cpp \
    Engine/IGeometry/ICollision/IProxyShape.cpp \
    Engine/IGeometry/ICollision/IRaycastInfo.cpp \
    Engine/IGeometry/ILight.cpp \
    Engine/IGeometry/IMateriall.cpp \
    Engine/IGeometry/IQCamera.cpp \
    Engine/IGeometry/IRect2D.cpp \
    Engine/IGeometry/Mesh/IMesh.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorBezierPatch.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorCapsule.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorCone.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorCuboid.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorCurve.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorCylinder.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorDetails.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorEllipsoid.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorPie.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorPipe.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorSpiral.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorTorus.cpp \
    Engine/IGeometry/Mesh/IMeshGeneratorTorusKnot.cpp \
    Engine/IGeometry/Mesh/IMeshLoader.cpp \
    Engine/IGeometry/Segments/ILine3D.cpp \
    Engine/IGeometry/Segments/ILineSegment3D.cpp \
    Engine/IGeometry/Segments/IPlane.cpp \
    Engine/IGeometry/Segments/IRay.cpp \
    Engine/IHierarchy/IHierarchy/IHierarchyNode.cpp \
    Engine/IHierarchy/IHierarchy/IHierarchyScene.cpp \
    Engine/IHierarchy/IHierarchyTransform.cpp \
    EngineComponent/IComponentAbstract.cpp \
    EngineComponent/IComponentCamera.cpp \
    EngineComponent/IComponentLight.cpp \
    EngineComponent/IComponentMesh.cpp \
    EngineComponent/IMaterial.cpp \
    GLWidget.cpp \
    Scene/IEngineFactory.cpp \
    Scene/IEngineGimbalStabilization.cpp \
    Scene/Robotics/VehicleRobotCar.cpp \
    Scene/SceneEngineNozzle.cpp \
    Scene/SceneEngineNuzzleGimbal.cpp \
    Scene/SceneEngineRobocar.cpp \
    Scene/SceneEngineTest.cpp \
    Scene/Sensors/ISensorEncoder.cpp \
    Scene/Sensors/ISensorLIDAR.cpp \
    Scene/Sensors/ISensorOrientation.cpp \
    fontprovider.cpp \
    main.cpp \
    mainwindow.cpp \
    Scene/IGizmo/IGizmoTransformMove.cpp \
    Scene/IGizmo/IGizmoTransformRender.cpp \
    Scene/IGizmo/IGizmoTransformRotate.cpp \
    Scene/IGizmo/IGizmoTransformScale.cpp \
    Scene/IGizmoManipulator.cpp \
    Scene/OpenGL/OpenGLRender.cpp \
    Scene/OpenGL/geometry_opengl.cpp \
    Scene/SceneEngine.cpp \
    Scene/SceneMain.cpp \
    Scene/Shader/Shader.cpp \
    qcustomplot.cpp

HEADERS += \
    Engine/IAlgorithm/GJK-EPA/IGjkEpa.h \
    Engine/IAlgorithm/IClliping.h \
    Engine/IAlgorithm/IQuickClipping.h \
    Engine/IAlgorithm/IQuickHull.hpp \
    Engine/IAlgorithm/IScanColsingPath.h \
    Engine/IAlgorithm/QuickHull/ConvexHull.hpp \
    Engine/IAlgorithm/QuickHull/HalfEdgeMesh.hpp \
    Engine/IAlgorithm/QuickHull/MathUtils.hpp \
    Engine/IAlgorithm/QuickHull/QuickHull.hpp \
    Engine/IAlgorithm/QuickHull/Structs/Mesh.hpp \
    Engine/IAlgorithm/QuickHull/Structs/Plane.hpp \
    Engine/IAlgorithm/QuickHull/Structs/Pool.hpp \
    Engine/IAlgorithm/QuickHull/Structs/Ray.hpp \
    Engine/IAlgorithm/QuickHull/Structs/Vector3.hpp \
    Engine/IAlgorithm/QuickHull/Structs/VertexDataSource.hpp \
    Engine/IAlgorithm/QuickHull/Types.hpp \
    Engine/IAlgorithm/iclliping3d.h \
    Engine/ICommon/IBernsteinPolynomial.h \
    Engine/ICommon/IBezierPatch.h \
    Engine/ICommon/IColor.h \
    Engine/ICommon/IMemory/IList.h \
    Engine/ICommon/IMemory/IMem.h \
    Engine/ICommon/IMemory/IPair.h \
    Engine/ICommon/IMemory/IStack.h \
    Engine/ICommon/ISettings.h \
    Engine/IComponent/IComponent.h \
    Engine/IDynamics/IConstraintSolver.h \
    Engine/IDynamics/IJoints/IBallAndSocketJoint.h \
    Engine/IDynamics/IJoints/IFixedJoint.h \
    Engine/IDynamics/IJoints/IHingeJoint.h \
    Engine/IDynamics/IJoints/IJoint.h \
    Engine/IDynamics/IJoints/ISliderJoint.h \
    Engine/IDynamics/IContactSolver.h \
    Engine/IDynamics/IDynamicsWorld.h \
    Engine/IDynamics/IIntegrateUtil.h \
    Engine/IDynamics/IIsland.h \
    Engine/IDynamics/IMaterial/IPhysicsMaterial.h \
    Engine/IDynamics/IRigidBody.h \
    Engine/IDynamics/IStep.h \
    Engine/IDynamics/ITimer.h \
    Engine/IGeometry/IAABBox3D.h \
    Engine/IGeometry/ICollision/IBody/IBody.h \
    Engine/IGeometry/ICollision/IBody/ICollisionBody.h \
    Engine/IGeometry/ICollision/IBroadPhase/IBroadPhase.h \
    Engine/IGeometry/ICollision/IBroadPhase/IDynamicAABBTree.h \
    Engine/IGeometry/ICollision/ICollisionContact/IContactGenerator.h \
    Engine/IGeometry/ICollision/ICollisionContact/IContactManager.h \
    Engine/IGeometry/ICollision/ICollisionContact/IContactManifold.h \
    Engine/IGeometry/ICollision/ICollisionContact/IContactManifoldSet.h \
    Engine/IGeometry/ICollision/ICollisionContact/IContactPoint.h \
    Engine/IGeometry/ICollision/ICollisionWorld.h \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShape.h \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShapeBox.h \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShapeConvex.h \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShapeHull.h \
    Engine/IGeometry/ICollision/ICollisionShapes/ICollisionShapeSphere.h \
    Engine/IGeometry/ICollision/ICollisionShapes/IConcaveMeshShape.h \
    Engine/IGeometry/ICollision/ICollisionShapes/IConcaveShape.h \
    Engine/IGeometry/ICollision/ICollisionShapes/IConvexPolyhedronShape.h \
    Engine/IGeometry/ICollision/ICollisionShapes/IHalfEdgeStructure.h \
    Engine/IGeometry/ICollision/ICollisionShapes/ITriangleMesh.h \
    Engine/IGeometry/ICollision/ICollisionShapes/ITriangleShape.h \
    Engine/IGeometry/ICollision/ICollisionShapes/ITriangleVertexArray.h \
    Engine/IGeometry/ICollision/INarrowPhase/GJK/IGJKIntersection.h \
    Engine/IGeometry/ICollision/INarrowPhase/GJK/Simplex.h \
    Engine/IGeometry/ICollision/INarrowPhase/ICollisionAlgorithm.h \
    Engine/IGeometry/ICollision/INarrowPhase/ICollisionAlgorithmGjkEpa.h \
    Engine/IGeometry/ICollision/INarrowPhase/ICollisionShapeInfo.h \
    Engine/IGeometry/ICollision/IOverlappingPair.h \
    Engine/IGeometry/ICollision/IProxyShape.h \
    Engine/IGeometry/ICollision/IRaycastInfo.h \
    Engine/IGeometry/ILight.h \
    Engine/IGeometry/IMateriall.h \
    Engine/IGeometry/IQCamera.h \
    Engine/IGeometry/IRect2D.h \
    Engine/IGeometry/Mesh/IIndexUtil.h \
    Engine/IGeometry/Mesh/IMesh.h \
    Engine/IGeometry/Mesh/IMeshGenerator.h \
    Engine/IGeometry/Mesh/IMeshGeneratorDetails.h \
    Engine/IGeometry/Mesh/IMeshLoader.h \
    Engine/IGeometry/Mesh/ITypes.h \
    Engine/IGeometry/Mesh/memory/dvector.h \
    Engine/IGeometry/Mesh/memory/iterator_util.h \
    Engine/IGeometry/Mesh/memory/refcount_vector.h \
    Engine/IGeometry/Mesh/memory/small_list_set.h \
    Engine/IGeometry/Segments/ILine3D.h \
    Engine/IGeometry/Segments/ILineSegment3D.h \
    Engine/IGeometry/Segments/IPlane.h \
    Engine/IGeometry/Segments/IRay.h \
    Engine/IGeometry/igeometry_types.hpp \
    Engine/IHierarchy/IHierarchy/IHierarchyNode.h \
    Engine/IHierarchy/IHierarchy/IHierarchyScene.h \
    Engine/IHierarchy/IHierarchyTransform.h \
    Engine/IMath/IAffineTransform.h \
    Engine/IMath/IAlgebra.h \
    Engine/IMath/IComplex.h \
    Engine/IMath/IFunc.h \
    Engine/IMath/ILorentzVector.h \
    Engine/IMath/IMatrix.h \
    Engine/IMath/IMatrix2x2.h \
    Engine/IMath/IMatrix3x3.h \
    Engine/IMath/IMatrix4x4.h \
    Engine/IMath/IOctonion.h \
    Engine/IMath/IQuaternion.h \
    Engine/IMath/IReal.h \
    Engine/IMath/IScalarType.h \
    Engine/IMath/ISpherical.h \
    Engine/IMath/ITransform.h \
    Engine/IMath/IVector.h \
    Engine/IMath/IVector2D.h \
    Engine/IMath/IVector3D.h \
    Engine/IMath/IVector4D.h \
    Engine/IMath/IVectorType.h \
    Engine/IMath/imaths.h \
    Engine/engine.hpp \
    Engine/imaths.hpp \
    EngineComponent/IComponentAbstract.h \
    EngineComponent/IComponentCamera.h \
    EngineComponent/IComponentLight.h \
    EngineComponent/IComponentMesh.h \
    EngineComponent/IEngineComponent.hpp \
    EngineComponent/IMaterial.h \
    GLWidget.h \
    Scene/IEngineFactory.h \
    Scene/IEngineGimbalStabilization.h \
    Scene/Robotics/VehicleRobotCar.h \
    Scene/SceneEngineNozzle.h \
    Scene/SceneEngineNuzzleGimbal.h \
    Scene/SceneEngineRobocar.h \
    Scene/SceneEngineTest.h \
    Scene/Sensors/ISensorEncoder.h \
    Scene/Sensors/ISensorLIDAR.h \
    Scene/Sensors/ISensorOrientation.h \
    fontprovider.h \
    mainwindow.h \
    Scene/IGizmo/IGizmoTransform.h \
    Scene/IGizmo/IGizmoTransformMove.h \
    Scene/IGizmo/IGizmoTransformRender.h \
    Scene/IGizmo/IGizmoTransformRotate.h \
    Scene/IGizmo/IGizmoTransformScale.h \
    Scene/IGizmo/IMathGizmo.h \
    Scene/IGizmo/IUGizmo.h \
    Scene/IGizmoManipulator.h \
    Scene/OpenGL/OpenGLRender.h \
    Scene/OpenGL/geometry_opengl.h \
    Scene/SceneEngine.h \
    Scene/SceneMain.h \
    Scene/Shader/Shader.h \
    qcustomplot.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \

RESOURCES += \
    fonts.qrc \
    shaders.qrc
