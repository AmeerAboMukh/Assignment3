


//#include <playsoundapi.h>
#include "BasicScene.h"


#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"

#include "igl/per_vertex_normals.h"
#include "igl/per_face_normals.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/edge_flaps.h"
#include "igl/loop.h"
#include "igl/upsample.h"
#include "igl/AABB.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/circulation.h"
#include "igl/edge_midpoints.h"
#include "igl/collapse_edge.h"
#include "igl/edge_collapse_is_valid.h"
#include "igl/write_triangle_mesh.h"
#include <random>
#include <count.h>
#include <forward_kinematics.h>
#include <dqs.h>
#include <directed_edge_parents.h>
//#include <Windows.h>
//#undef max
//#include <mmsystem.h>
//#pragma comment(lib, "WinMM.lib")
// #include "AutoMorphingModel.h"

using namespace cg3d;

void BasicScene::RestartTheSnakeToInit() {
    
    cyls[0]->TranslateInSystem(root->GetRotation(), -(cyls[0]->GetTranslation()));
    for (int i = 0; i < cyls.size(); i++) {
        cyls[i]->Rotate(cyls[i]->GetRotation().inverse());
    }
    cyls[0]->RotateByDegree(-90, Axis::X);
    
    auto program = std::make_shared<Program>("shaders/phongShader");
    auto material{ std::make_shared<Material>("material", program) };
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj") };
    optCyl = Model::Create("cyl", cylMesh, material);
    optCyl->Translate(1.6f * (linksSize - 2), Axis::Z);
    optCyl->SetCenter(Eigen::Vector3f(0, 0, -0.8f * 1));
    //optCyl->isHidden = true;
    root->AddChild(optCyl);
    optCyl->Scale(0.09f);
    root->RemoveChild(snake);
    auto snakemesh{ IglLoader::MeshFromFiles("snake","data/snake1.obj") };
    snake = Model::Create("snake", snakemesh, material);
    snake->showWireframe = true;
    snake->showFaces = false;
    root->AddChild(snake);
    snake->Translate(1.6 * 3 + 0.4, Axis::Z);
   /* C = Eigen::MatrixXd(linksSize + 1, 3);
    BE = Eigen::MatrixXi(linksSize, 2);
    C.row(0) << 0, 0, -1.6 * (linksSize / 2);
    BE.row(0) << 0, 1;
    for (int i = 1; i < linksSize; i++){
        C.row(i) << 0, 0, (1.6)* ((i - (linksSize / 2.0f)));
        BE.row(i) << i, i + 1;
    }
    C.row(linksSize) << 0, 0, (1.6)* ((linksSize / 2));
    V = scaled(snakemesh->data[0].vertices, { 1, 1,  linksSize });
    F = snakemesh->data[0].faces;
    U = V;
    igl::directed_edge_parents(BE, P);
    W = Eigen::MatrixXd::Zero(V.rows(), C.rows() - 1);
    for (size_t i = 0; i < V.rows(); i++)
    {
        Eigen::Vector3f v = V.row(i).cast<float>().eval();
        Weights(v, C,i);
    }
    }
    */
}

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
   cameraslist.push_back(Camera::Create("camera1", fov, float(width) / height,  near, far));
   cameraslist.push_back(Camera::Create("camera2", fov, float(width) / height, near, far));
   cameraslist.push_back(Camera::Create("camera1", fov, float(width) / height, near, far));
   cameraslist.push_back(Camera::Create("camera2", fov, float(width) / height, near, far));
 
   
   camera = cameraslist[0];

    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    treeofSphere.resize(10);
    treeofCyls.resize(linksSize);
    treeofbombers.resize(2);
    treeofgifts.resize(5);
    BoxofCyls.resize(5);
    BoxofCyls.resize(4);

    auto program = std::make_shared<Program>("shaders/phongShader");

    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    auto program2 = std::make_shared<Program>("shaders/basicShader");

    auto material{ std::make_shared<Material>("material", program) }; // empty material
    auto material1{ std::make_shared<Material>("material", program1) };
    auto material2{ std::make_shared<Material>("material", program2) };// empty material

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{ IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") };
    auto cylMesh{ IglLoader::MeshFromFiles("cyl_igl","data/zcylinder.obj") };
    auto garceMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj") };
    auto cubeMesh{ IglLoader::MeshFromFiles("cube_igl","data/cube_old.obj") };
    auto snakemesh{ IglLoader::MeshFromFiles("snake","data/snake1.obj") };
    auto giftsMesh{ IglLoader::MeshFromFiles("ball_igl","data/cube_old.obj") };
    material2->AddTexture(0, "textures/grass.bmp",2);
     grace1 = Model::Create("grace", Mesh::Cube(), material2);
     grace1->Scale(100);
     grace1->Translate(-20.f, Axis::Y);
     root->AddChild(grace1);
     snake = Model::Create("snake", snakemesh, material);
     snake->showWireframe = true;
     snake->showFaces = false;
     root->AddChild(snake);
     C = Eigen::MatrixXd(linksSize + 1, 3);
     BE = Eigen::MatrixXi(linksSize, 2);
     C.row(0) << 0, 0, -1.6 * (linksSize / 2);
     BE.row(0) << 0, 1;
    
    for (int i = 0; i < 10; i++) {
        sphere1 = Model::Create("sphere", sphereMesh, material);
        root->AddChild(sphere1);
        spehersforlevel1.push_back(sphere1);
        treeofSphere[i].init(sphere1->GetMeshList()[0]->data[0].vertices, sphere1->GetMeshList()[0]->data[0].faces);
    }
    for (int i = 0; i < 5; i++) {
        gift = Model::Create("gift", giftsMesh, material);
        root->AddChild(gift);
        gifts.push_back(gift);
        treeofgifts[i].init(gift->GetMeshList()[0]->data[0].vertices, gift->GetMeshList()[0]->data[0].faces);
    }
    for (int i = 0; i < 2; i++) {
        bomber = Model::Create("bomber", cubeMesh, material);
        root->AddChild(bomber);
        bombers.push_back(bomber);
        treeofbombers[i].init(bomber->GetMeshList()[0]->data[0].vertices, bomber->GetMeshList()[0]->data[0].faces);
    }

    //Axis
    Eigen::MatrixXd vertices(6, 3);
    vertices << -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;
    Eigen::MatrixXi faces(3, 2);
    faces << 0, 1, 2, 3, 4, 5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6, 3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6, 2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);
    axis.push_back(Model::Create("axis", coordsys, material1));
    axis[0]->mode = 1;
    axis[0]->Scale(2, Axis::XYZ);
    axis[0]->isHidden = true;
    root->AddChild(axis[0]);
    float scaleFactor = 1;
    cyls.push_back(Model::Create("cyl", cylMesh, material));
    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    snake->Translate(1.6*3+0.4, Axis::Z);
    cyls[0]->isHidden = true;
    root->AddChild(cyls[0]);
   // cyls[0]->AddChild(snake);
    //snake->AddChild(cyls[0]);
    //cyls[cyls.size() - 1]->AddChild(snake);
   treeofCyls[0].init(cyls[0]->GetMeshList()[0]->data[0].vertices, cyls[0]->GetMeshList()[0]->data[0].faces);
    for (int i = 1; i < linksSize; i++)
    {
        cyl = Model::Create("cyl", cylMesh, material);
        cyls.push_back(cyl);
        cyls[i]->Translate(1.6f * scaleFactor, Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
        cyls[i - 1]->AddChild(cyls[i]);
        cyls[i]->isHidden = true;
        
        axis.push_back(Model::Create("axis", coordsys, material1));
        axis[i]->mode = 1;
        axis[i]->Scale(2, Axis::XYZ);
        cyls[i - 1]->AddChild(axis[i]);
        axis[i]->Translate({ 0,0,0.8f });
        axis[i]->isHidden = true;
        treeofCyls[i].init(cyl->GetMeshList()[0]->data[0].vertices, cyl->GetMeshList()[0]->data[0].faces);
        C.row(i) << 0, 0, (1.6)* ((i - (linksSize / 2.0f)));
        BE.row(i) << i, i + 1;
    }
    cyls[cyls.size() - 2]->AddChild(cameraslist[3]);
    optCyl = Model::Create("cyl", cylMesh, material);
    optCyl->Translate(1.6f * (linksSize-2), Axis::Z);
    optCyl->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    //optCyl->isHidden = true;
    optCyl->Scale(0.09f);
    root->AddChild(optCyl);
    cyls[1]->AddChild(cameraslist[1]);
   
    root->RotateByDegree(-90, Axis::X);
    //snake->Scale(1.6 * 3 - 0.8, Axis::Z);
    
    auto morphFunc = [](Model* model, cg3d::Visitor* visitor) {
        return model->meshIndex;//(model->GetMeshList())[0]->data.size()-1;
    };

     C.row(linksSize) << 0, 0, (1.6)* ((linksSize / 2));
    V = scaled(snakemesh->data[0].vertices, { 1, 1 , 1.6 * linksSize / 1.6 });
    F = snakemesh->data[0].faces;
    U = V;
    igl::directed_edge_parents(BE, P);
    W = Eigen::MatrixXd::Zero(V.rows(), C.rows() - 1);
    for (size_t i = 0; i < V.rows(); i++)
    {
        Eigen::Vector3f v = V.row(i).cast<float>().eval();
        Weights(v, C,i);
    }
    
    root->AddChild(cameraslist[1]);
    root->AddChild(cameraslist[2]);
   
    
    cameraslist[0]->Translate(60, Axis::Z);
    cameraslist[1]->RotateByDegree(180.f, Axis::Y);
    cameraslist[1]->Translate(-35, Axis::Z);
    cameraslist[2]->RotateByDegree(180.f, Axis::Z);
    cameraslist[2]->Translate(40, Axis::Z);
    cameraslist[3]->RotateByDegree(-155, Axis::X);
    cameraslist[3]->RotateByDegree(180, Axis::Z);
    cameraslist[3]->Translate(5, Axis::Y);
    cameraslist[3]->Translate(-8, Axis::Z);
    
}


void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    // cube->Rotate(0.1f, Axis::XYZ);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0f);
    program.SetUniform4f("light_position", 0.0, 15.0f, 0.0, 1.0f);
   // autoCube->Translate(0,Axis::XYZ);
    sphere1->Translate(0,Axis::XYZ);
    
    if(playnow){
    if (remainning <=0) {
        remainning += 1000 ;
        state = Statue::Looser;
    }
    if(!selfColl){   
        SelfCollision();
    }
    if((DK || AK || WK || SK) &&( HardGame || train)){
    Eigen::Vector3f norm(0, 0, 1);
    norm = root->GetRotation().inverse() * optCyl->GetRotation() * norm;
    norm = norm.normalized() * speed;
    optCyl->Translate(norm);
    Eigen::Vector3f Pos = Eigen::Vector3f::Zero();
    Eigen::Matrix3f rotat = optCyl->GetRotation();
    Eigen::Vector3f Place = optCyl->GetAggregatedTransform().block<3, 1>(0, 3);
     source = ((-1 * rotat * half_length) + Place);
    FabrikMethodHardGame(source);
    }
   /* if (AK) {
        if (iA == 1) {
            cyls[cyls.size() - 1]->Translate(-0.005, Axis::X);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }if (iA == 2) {
            cyls[cyls.size() - 1]->Translate(-0.005, Axis::Z);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }if (iA == 3) {
            cyls[cyls.size() - 1]->Translate(+0.005, Axis::X);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }if (iA == 0) {
            cyls[cyls.size() - 1]->Translate(0.005, Axis::Z);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }
    }
    if (DK) {
        if (iD == 1) {
            cyls[cyls.size() - 1]->Translate(0.005, Axis::X);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }if (iD == 2) {
            cyls[cyls.size() - 1]->Translate(-0.005, Axis::Z);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }if (iD == 3) {
            cyls[cyls.size() - 1]->Translate(-0.005, Axis::X);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }if (iD == 0) {
            cyls[cyls.size() - 1]->Translate(0.005, Axis::Z);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }
    }if (WK) {
        if (jW == 0) {
            cyls[cyls.size() - 1]->Translate(0.005, Axis::Y);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }//if (jW == 2) {
           // cyls[cyls.size() - 1]->Translate(-0.005, Axis::Z);
           // source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
           // FabrikMethodHardGame(source);
    //    }
    if (jW == 1) {
            cyls[cyls.size() - 1]->Translate(-0.005, Axis::Y);
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }//if (jW == 0) {
           // cyls[cyls.size() - 1]->Translate(0.005, Axis::Z);
           // source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            //FabrikMethodHardGame(source);
        //}
    }if (SK) {

        if (jS == 0) {
            cyls[cyls.size() - 1]->Translate(-0.005, Axis::Y);           
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }//if (jS == 2) {
           // cyls[cyls.size() - 1]->Translate(-0.005, Axis::Z);
           // source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            //FabrikMethodHardGame(source);
    //    }a
    if (jS == 1) {
            cyls[cyls.size() - 1]->Translate(0.005, Axis::Y);  
            source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            FabrikMethodHardGame(source);
        }//if (jS == 0) {
           // cyls[cyls.size() - 1]->Translate(0.005, Axis::Z);
            //source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
            //FabrikMethodHardGame(source);
       // }
    }if (QK) {
        cyls[cyls.size() - 1]->Translate(0.0001, Axis::Y);
        source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
        FabrikMethodHardGame(source);
    }if (EK) {
        cyls[cyls.size() - 1]->Translate(-0.0001, Axis::Y);
        source = GetTipPosOrGetSourcePos(cyls.size() - 1, -1);
        FabrikMethodHardGame(source);
    }*/

      if (remainning > 0 && !winner) {
        if (easyGame && CurrentLevel == 1 && DeterScore1 <=  score && !train) {
            remainning += 1000;
            winner = true;
            state = Statue::Winner;
        }if (easyGame && CurrentLevel == 2 && DeterScore2 <=  score && !train) {
            remainning += 1000;
            winner = true;
            state = Statue::Winner;
        }if (easyGame && CurrentLevel == 3 && DeterScore3 <=  score && !train) {
            remainning += 1000;
            winner = true;
            state = Statue::Winner;
        }if (HardGame && CurrentLevel == 1 && DeterScore4 <=  score && !train) {
            remainning += 1000;
            winner = true;
            state = Statue::Winner;
        }if (HardGame && CurrentLevel == 2 && DeterScore5 <=  score && !train) {
            remainning += 1000;
            winner = true;
            state = Statue::Winner;
        }if (HardGame && CurrentLevel == 1 && DeterScore6 <=  score && !train) {
            remainning += 1000;
            winner = true;
            state = Statue::Winner;
        }
    }
    if (pickedModel != nullptr && easyGame && CurrentLevel ==1 ) {
        FabrikMethodEasyGame(pickedModel->GetAggregatedTransform().block<3, 1>(0, 3),30.f);
    }else if (pickedModel != nullptr && easyGame && CurrentLevel == 2) {
        FabrikMethodEasyGame(pickedModel->GetAggregatedTransform().block<3, 1>(0, 3),40.f);
    }else if (pickedModel != nullptr && easyGame && CurrentLevel == 3) {
        FabrikMethodEasyGame(pickedModel->GetAggregatedTransform().block<3, 1>(0, 3),50.f);
    }
    if (HardGame && CurrentLevel == 1 ||train) {
        CollisionoftargetHard(indexofColl+1, 10);
            Collisionofgift(2, 15);
        }
    if (HardGame && CurrentLevel == 2) {
        CollisionoftargetHard(indexofColl + 1, 10);
       
    }
    if (HardGame && CurrentLevel == 3) {
        CollisionoftargetHard(indexofColl + 1, 10);
        Collisionofgift(1, 15);
        Collisionofbombers(2);
    }
    }


   // cyls[0]->TranslateInSystem(Eigen::Affine3f(cyls[cyls.size() - 1]->GetAggregatedTransform()).rotation().transpose(), Eigen::Vector3f(0, 0, 0.004f));
    /*for (int i = 0; i < spehersforlevel1.size(); i++) {
        for (int j = 3; j < cyls.size(); j++) {
            bool x = Collision(&treeofCyls[j], &treeofSphere[i], i, j);
            if (x && stat ) {
                stat = false;
                std::cout << "Collision Detected! " << i << " : " << 1 << std::endl;

            }
        }
    }*/
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }
    // note: there's a (small) chance the button state here precedes the mouse press/release event

    if (action == GLFW_PRESS) { // default mouse button press behavior
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;
        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel)
            pickedToutAtPress = pickedModel->GetTout();
        else
            cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    auto system = camera->GetRotation().transpose();
   
    bool enter = false;
        for (int i = 0; i < cyls.size(); i++) {
            if (pickedModel == cyls[i]) {
                enter = true;
                break;
            }
        }
        if (enter && pickedModel) {
            cyls[0]->TranslateInSystem(cyls[0]->GetRotation() * system, {0, 0, -float(yoffset)});
            pickedToutAtPress = pickedModel->GetTout();
        }
         else if (pickedModel && !enter){
            pickedModel->TranslateInSystem(pickedModel->GetRotation() * system, {0, 0, -float(yoffset)});
            pickedToutAtPress = pickedModel->GetTout();
         } else{
            root->TranslateInSystem(system, {0, 0, -float(yoffset)});
           // cameraToutAtPress = camera->GetTout();
         }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }
    if (dragging) {
        auto system = camera->GetRotation().transpose() * GetRotation();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        bool enter = false;
        if (pickedModel) {
            for (int i = 0; i < cyls.size(); i++) {
                if (pickedModel == cyls[i]) {
                    enter = true;
                    break;
                }
            }
            //pickedModel->SetTout(pickedToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE && enter) 
                cyls[0]->TranslateInSystem(cyls[0]->GetRotation() * system, {-float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0});
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE && !enter)
                pickedModel->TranslateInSystem( pickedModel->GetRotation() * system, { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
               pickedModel->RotateInSystem(axis[pickedIndex]->GetTout().rotation(), -float(xAtPress - x) / angleCoeff, Axis::Z);
                pickedModel->RotateInSystem(pickedModel->GetRotation() * system, -float(yAtPress - y) / angleCoeff, Axis::X);
            }
        } else {
           // camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                root->TranslateInSystem(system, {-float(xAtPress - x) / moveCoeff/10.0f, float( yAtPress - y) / moveCoeff/10.0f, 0});
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                root->RotateInSystem(system, float(x - xAtPress) / 180.0f, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                root->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                root->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
        xAtPress =  x;
        yAtPress =  y;
    }
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();
    bool enter;
    Eigen::Vector3f des;
    Eigen::Vector3f TipCyl = Eigen::Vector3f::Zero();
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
            case GLFW_KEY_SPACE:
                if (easyGame && pickedModel != nullptr  ) {
                    Eigen::Vector3f D = pickedModel->GetAggregatedTransform().block<3, 1>(0, 3);//target Position
                }
                break;
            case GLFW_KEY_1:
                camera = cameraslist[0];
                viewport->camera = camera;
                break;
            case GLFW_KEY_2:
                camera = cameraslist[1];
                viewport->camera = camera;
                break;
            case GLFW_KEY_3:
                camera = cameraslist[2];
                viewport->camera = camera;
                break;
            case GLFW_KEY_4:
                camera = cameraslist[3];
                viewport->camera = camera;
                break;
            case GLFW_KEY_W:
               if(HardGame || train){
                jW = (jW + 1) % 2;
                if (jW == 0) {
                    jS = 1;
                }
                else if (jW == 1) {
                    jS = 0;
                }
            
                animation = true;
                WK = true;
                QK = false;
                EK = false;
                AK = false; 
                SK = false;
                DK = false;
                optCyl->RotateByDegree(-15.f, Axis::X);
               }
            
                break;
            case GLFW_KEY_A:
                if (HardGame || train) {
                    iA = (iA + 1) % 4;

                    if (iA == 0) {
                        iD = 0;
                    }
                    else if (iA == 1) {
                        iD = 3;
                    }
                    else if (iA == 2) {
                        iD = 2;
                    }
                    else {
                        iD = 1;
                    }
                    animation = true;
                    WK = false;
                    QK = false;
                    EK = false;
                    AK = true;
                    SK = false;
                    DK = false;
                    optCyl->RotateByDegree(-15.f, Axis::Y);
                }
            
                break;
            case GLFW_KEY_D:
                if (HardGame || train) {
                    iD = (iD + 1) % 4;
                    if (iD == 0) {
                        iA = 0;
                    }
                    else if (iD == 1) {
                        iA = 3;
                    }
                    else if (iD == 2) {
                        iA = 2;
                    }
                    else {
                        iA = 1;
                    }

                    animation = true;
                    WK = false;
                    QK = false;
                    EK = false;
                    AK = false;
                    SK = false;
                    DK = true;
                    optCyl->RotateByDegree(15.f, Axis::Y);
                  
                }
                break;
            case GLFW_KEY_S:
                if (HardGame || train) {
                    jS = (jS + 1) % 2;
                    if (jS == 0) {
                        jW = 1;
                    }
                    else if (jS == 1) {
                        jW = 0;
                    }
                    // else if (jS == 2) {
                      //   jW = 2;
                    // }
                    // else {
                      //   jW = 1;
     //                }
                    animation = true;
                    WK = false;
                    QK = false;
                    EK = false;
                    AK = false;
                    SK = true;
                    DK = false;
                    optCyl->RotateByDegree(15.f, Axis::X);
                }
               // cyls[0]->Translate(-1 * speed, Axis::Z);
/*snake->Translate(-1 * speed, Axis::Z);
               source = GetTipPosOrGetSourcePos(cyls.size() - 1, 1);
                source = Eigen::Vector3f(source.x(), source.y() - 5, source.z());
                FabrikMethodHardGame(source);*/
                break;
         
        }
    }
}
void BasicScene::OurRotate(int mod, float number1, float number2) {
    Eigen::Matrix3f Rotation = pickedModel->GetRotation();
    Eigen::Vector3f euler_angle = Rotation.eulerAngles(2, 0, 2);
    float phi = euler_angle[0];
    float theta = euler_angle[1];
    float psi = euler_angle[2];
    Eigen::Matrix3f phiMatrix = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f thetaMatrix = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f psiMatrix = Eigen::Matrix3f::Zero();
    if (mod == 0) {//up
        theta -= number1;
    }
    else if (mod == 1) {//right
        phi -= number1;
    }
    else if (mod == 2) {//left
        phi += number1;
    }
    else if (mod == 3) {//down
        theta += number1;
    }
    else {//left mouse 
        phi += number1;
        theta += number2;
    }
    phiMatrix << cos(phi), -1 * sin(phi), 0, sin(phi), cos(phi), 0, 0, 0, 1;
    thetaMatrix << 1, 0, 0, 0, cos(theta), -1 * sin(theta), 0, sin(theta), cos(theta);
    psiMatrix << cos(psi), -1 * sin(psi), 0, sin(psi), cos(psi), 0, 0, 0, 1;
    pickedModel->Rotate(Rotation.transpose() * (phiMatrix * thetaMatrix * psiMatrix));
}
Eigen::Vector3f BasicScene::GetSpherePos()
{
    Eigen::Vector3f l = Eigen::Vector3f(1.6f, 0, 0);
    Eigen::Vector3f res;
    res = cyls[tipIndex]->GetRotation() * l;
    return res;
}
Eigen::Vector3f BasicScene::GetTipPosOrGetSourcePos(int CylIndex, int mod) {
    Eigen::Vector3f Pos = Eigen::Vector3f::Zero();
    Eigen::Matrix3f rotat = cyls[CylIndex]->GetRotation();
    if (mod == 1 || mod == -1) {
        Eigen::Vector3f Place = cyls[CylIndex]->GetAggregatedTransform().block<3, 1>(0, 3);
        return ((mod * rotat * half_length) + Place);
    }
    else {
        return Pos;
    }
}
void BasicScene::FabrikMethodEasyGame(Eigen::Vector3f D, float x) {
    if (animation) {
       // Eigen::Vector3f D = sphere1->GetAggregatedTransform().block<3, 1>(0, 3);//target Position
        Eigen::Vector3f Source_Position_of_The_first_Cyl = GetTipPosOrGetSourcePos(0, -1);//root
        std::vector<Eigen::Vector3f> p;
        p.resize(cyls.size() + 1);
        for (int i = 0; i <= cyls.size() - 1; i++) {
            p[i] = GetTipPosOrGetSourcePos(i, -1);
        }
        p[cyls.size()] = GetTipPosOrGetSourcePos(cyls.size() - 1, 1);
        std::vector<double> ri_array;
        std::vector<double> lambda_i_array;
        ri_array.resize(cyls.size() + 1);
        lambda_i_array.resize(cyls.size() + 1);
        Eigen::Vector3f b = p[0];
        Eigen::Vector3f endEffector = p[cyls.size()];
        float diff_A = (endEffector - D).norm();
    
        while (diff_A > 0.05) {
            p[cyls.size()] = D;
            int i = cyls.size();
            while (i - 1 != -1) {
                ri_array[i - 1] = (p[i] - p[i - 1]).norm();
                lambda_i_array[i - 1] = 1.6 / ri_array[i - 1];
                p[i - 1] = (1 - lambda_i_array[i - 1]) * p[i] + lambda_i_array[i - 1] * p[i - 1];
                i = i - 1;  
            }
            diff_A = (p[cyls.size()] - D).norm();
        }
        for (int i = 0; i < cyls.size(); i++) {
            Eigen::Vector3f R = GetTipPosOrGetSourcePos(i, -1);//source current cyl
            Eigen::Vector3f E = GetTipPosOrGetSourcePos(cyls.size() - 1, 1);//Tip Last cyl
            Eigen::Vector3f RD = (D - R).normalized(); // distance between target position and source of current cyl
            Eigen::Vector3f RE = (E - R).normalized(); // distance between Tip Last cyl and source of current cyl
            float dot = RD.dot(RE);
            Eigen::Vector3f norm = cyls[i]->GetRotation().transpose() * ((RE.cross(RD)));
            cyls[i]->Rotate(((acosf(std::clamp(dot, -1.0f, 1.0f))) / x), norm);
        }
        if (((GetTipPosOrGetSourcePos(0, -1) - p[0]).norm() / 10) > 0.1f){
            cyls[0]->Translate(root->GetRotation().inverse() * ((GetTipPosOrGetSourcePos(0, -1) - p[0]).normalized() * -float(0.01f)));
            snake->Translate(root->GetRotation().inverse() * ((GetTipPosOrGetSourcePos(0, -1) - p[0]).normalized() * -float(0.01f)));
                
        }
        else {
            cyls[0]->Translate(root->GetRotation().inverse() * ((GetTipPosOrGetSourcePos(0, -1) - p[0]).normalized() * -float((GetTipPosOrGetSourcePos(0, -1) - p[0]).norm() / 100)));
            snake->Translate(root->GetRotation().inverse() * ((GetTipPosOrGetSourcePos(0, -1) - p[0]).normalized() * -float((GetTipPosOrGetSourcePos(0, -1) - p[0]).norm() / 100)));
        }
        if ((D - GetTipPosOrGetSourcePos(cyls.size() - 1, 1)).norm() < 0.5) {
            if (easyGame && CurrentLevel == 1) {
                if (train) {
                    Collisionoftarget(10, 0);
                    Collisionofgift(1, 0);
                }
                else {
                    Collisionoftarget(10, 10);
                    Collisionofgift(1, 15);
                }
            }
            else if (easyGame && CurrentLevel == 2) {
                Collisionoftarget(5, 10);
                Collisionofgift(1, 15);
            }
            else if (easyGame && CurrentLevel == 3) {
                Collisionoftarget(3, 10);
                Collisionofgift(1, 15);
                Collisionofbombers(2);

            }
            
        }
        skinning();

    }
}

void BasicScene::FabrikMethodHardGame(Eigen::Vector3f D) {
    if (animation) {
        // Eigen::Vector3f D = sphere1->GetAggregatedTransform().block<3, 1>(0, 3);//target Position
       Eigen::Vector3f Source_Position_of_The_first_Cyl = GetTipPosOrGetSourcePos(0, -1);//root
        std::vector<Eigen::Vector3f> p;
        p.resize(cyls.size() + 1);
        for (int i = 0; i <= cyls.size() - 1; i++) {
            p[i] = GetTipPosOrGetSourcePos(i, -1);
        }
        p[cyls.size()] = GetTipPosOrGetSourcePos(cyls.size() - 1, 1);
        std::vector<double> ri_array;
        std::vector<double> lambda_i_array;
        ri_array.resize(cyls.size() + 1);
        lambda_i_array.resize(cyls.size() + 1);
        Eigen::Vector3f b = p[0];
        Eigen::Vector3f endEffector = p[cyls.size()];
        float diff_A = (endEffector - D).norm();
    
        while (diff_A > 0.05) {
            p[cyls.size()] = D;
            int i = cyls.size();
            while (i - 1 != -1) {
                ri_array[i - 1] = (p[i] - p[i - 1]).norm();
                lambda_i_array[i - 1] = 1.6 / ri_array[i - 1];
                p[i - 1] = (1 - lambda_i_array[i - 1]) * p[i] + lambda_i_array[i - 1] * p[i - 1];
                i = i - 1;  
            }
            diff_A = (p[cyls.size()] - D).norm();
        }
        for (int i = 0; i < cyls.size(); i++) {
            Eigen::Vector3f R = GetTipPosOrGetSourcePos(i, -1);//source current cyl
            Eigen::Vector3f E = GetTipPosOrGetSourcePos(cyls.size() - 1, 1);//Tip Last cyl
            Eigen::Vector3f RD = (D - R).normalized(); // distance between target position and source of current cyl
            Eigen::Vector3f RE = (E - R).normalized(); // distance between Tip Last cyl and source of current cyl
            float dot = RD.dot(RE);
            Eigen::Vector3f norm = cyls[i]->GetRotation().transpose() * ((RE.cross(RD)));
            cyls[i]->Rotate(((acosf(std::clamp(dot, -1.0f, 1.0f))) / 30.f), norm);
        }
        if (((GetTipPosOrGetSourcePos(0, -1) - p[0]).norm() / 10) > 0.1f){
            cyls[0]->Translate(root->GetRotation().inverse() * ((GetTipPosOrGetSourcePos(0, -1) - p[0]).normalized() * -float(0.01f)));
            snake->Translate(root->GetRotation().inverse() * ((GetTipPosOrGetSourcePos(0, -1) - p[0]).normalized() * -float(0.01f)));
                
        }
        else {
            cyls[0]->Translate(root->GetRotation().inverse() * ((GetTipPosOrGetSourcePos(0, -1) - p[0]).normalized() * -float((GetTipPosOrGetSourcePos(0, -1) - p[0]).norm() / 100)));
            snake->Translate(root->GetRotation().inverse() * ((GetTipPosOrGetSourcePos(0, -1) - p[0]).normalized() * -float((GetTipPosOrGetSourcePos(0, -1) - p[0]).norm() / 100)));
        }
        if ((D - GetTipPosOrGetSourcePos(cyls.size() - 2, 1)).norm() < 0.0005) {
            animation = false;
        }
        
        skinning();

    }
}

void BasicScene::CCDMethod() {
    if (animation) {
        Eigen::Vector3f D = sphere1->GetAggregatedTransform().block<3, 1>(0, 3);//target Position
        Eigen::Vector3f Source_Position_of_The_first_Cyl = GetTipPosOrGetSourcePos(0, -1);
        float Length_From_First_cyl_To_des = (D - Source_Position_of_The_first_Cyl).norm();
        bool Can_Reach = (Length_From_First_cyl_To_des <= 1.6 * cyls.size()) ? true : false;
        for (int i = cyls.size() - 1; i >= 0 && Can_Reach; i--) {
            Eigen::Vector3f R = GetTipPosOrGetSourcePos(i, -1);//source current cyl
            Eigen::Vector3f E = GetTipPosOrGetSourcePos(cyls.size() - 1, 1);//Tip Last cyl
            Source_Position_of_The_first_Cyl = GetTipPosOrGetSourcePos(0, -1);
            D = sphere1->GetAggregatedTransform().block<3, 1>(0, 3);//target Position

            if ((D - E).norm() < 0.05) { // condition number 3
                animation = false;
                std::cout << "---------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
                std::cout << "distance between target position and tip of last cyl is :- " << (D - E).norm() << std::endl;
                std::cout << "---------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;

                break;
            }
            Eigen::Vector3f RD = (D - R).normalized(); // distance between target position and source of current cyl
            Eigen::Vector3f RE = (E - R).normalized(); // distance between Tip Last cyl and source of current cyl
            float dot = RD.dot(RE);

            Eigen::Vector3f norm = cyls[i]->GetRotation().transpose() * ((RE.cross(RD)));
            cyls[i]->Rotate(((acosf(std::clamp(dot, -1.0f, 1.0f))) / 35.f), norm);
        }
        if (!Can_Reach) {
            std::cout << "---------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
            std::cout << "Cannot Reach!! The length is :- " << (Length_From_First_cyl_To_des) << std::endl;
            std::cout << "---------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;

            animation = false;
        }
        
    }

}


bool BasicScene::Collision(igl::AABB <Eigen::MatrixXd, 3>* tree1, igl::AABB <Eigen::MatrixXd, 3>* tree2, int index1, int index2, std::vector<std::shared_ptr<cg3d::Model>> x)
{

    if (!Collision_Conditions(tree1->m_box, tree2->m_box , index1,index2,x)) {
        return false;
    }
    else {
        if (tree1->is_leaf() && tree2->is_leaf()) {
            return true;
        }
        igl::AABB<Eigen::MatrixXd, 3>* lefttree1 = tree1->is_leaf() ? tree1 : tree1->m_left;
        igl::AABB<Eigen::MatrixXd, 3>* righttree1 = tree1->is_leaf() ? tree1 : tree1->m_right;
        igl::AABB<Eigen::MatrixXd, 3>* lefttree2 = tree2->is_leaf() ? tree2 : tree2->m_left;
        igl::AABB<Eigen::MatrixXd, 3>* righttree2 = tree2->is_leaf() ? tree2 : tree2->m_right;

        if (Collision(lefttree1, lefttree2, index1, index2,x) || Collision(lefttree1, righttree2, index1, index2,x) ||
            Collision(righttree1, lefttree2, index1, index2,x) || Collision(righttree1, righttree2, index1, index2,x)) {
            return true;
        }
        else
            return false;
    }
}

bool BasicScene::Collision_Conditions(Eigen::AlignedBox<double, 3> Frame1, Eigen::AlignedBox<double, 3> Frame2, int index1, int index2 , std::vector<std::shared_ptr<cg3d::Model>> x) {
    double R0, R1, R;
    Eigen::Matrix3d A =cyls[index2]->GetRotation().cast<double>();
    Eigen::Matrix3d B = x[index1]->GetRotation().cast<double>();
    Eigen::Matrix3d C = A.transpose() * B;
    Eigen::Vector3d CenterOfFrame1 = Frame1.center();
    Eigen::Vector3d CenterOfFrame2 = Frame2.center();
    //Eigen::Vector4d x = cyls[index2]->GetAggregatedTransform().cast<double>() * Eigen::Vector4d(CenterOfFrame1(0), CenterOfFrame1(1), CenterOfFrame1(2), 1);
    //CenterOfFrame1 = x.head<3>();
    Eigen::Vector4d CenterofA = Eigen::Vector4d(CenterOfFrame1[0], CenterOfFrame1[1], CenterOfFrame1[2], 1);
    Eigen::Vector4d CenterofB = Eigen::Vector4d(CenterOfFrame2[0], CenterOfFrame2[1], CenterOfFrame2[2], 1);
   // Eigen::Affine3f scaling1 = spehersforlevel1[index1]->GetScaling(spehersforlevel1[index1]->GetTransform());
   // Eigen::Affine3f scaling2 = snake->GetScaling(snake->GetTransform());
   // auto ScaleA = spehersforlevel1[index1]->GetScaling(spehersforlevel1[index1]->GetTransform())(0,0);
    //auto ScaleB = snake->GetScaling(snake->GetTransform())(0,0);
    //std::cout << "-------- " << ScaleB << std::endl;
    double a0 =   (Frame1.sizes()[0] / 2);
    double a1 =   (Frame1.sizes()[1] / 2);
    double a2 =   (Frame1.sizes()[2] / 2);
    double b0 =   (Frame2.sizes()[0] / 2);
    double b1 =  (Frame2.sizes()[1] / 2);
    double b2 =   (Frame2.sizes()[2] / 2);
    Eigen::RowVector3d A0 = A.col(0).transpose();
    Eigen::RowVector3d A1 = A.col(1).transpose();
    Eigen::RowVector3d A2 = A.col(2).transpose();
    Eigen::RowVector3d B0 = B.col(0).transpose();
    Eigen::RowVector3d B1 = B.col(1).transpose();
    Eigen::RowVector3d B2 = B.col(2).transpose();
    Eigen::Vector4d newCenterOfB = x[index1]->GetAggregatedTransform().cast<double>() * CenterofB;
    //std::cout << "-------- " << cyls[index2]->GetTransform().cast<double>() << std::endl;

    Eigen::Vector4d newCenterOfA = cyls[index2]->GetAggregatedTransform().cast<double>() * CenterofA;
    Eigen::Vector4d Current_D = newCenterOfB - newCenterOfA;
    Eigen::Vector3d D = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; i++) {
        D[i] = Current_D[i];
    }

    R0 = a0;
    R1 = b0 * abs(C.row(0)[0]) + b1 * abs(C.row(0)[1]) + b2 * abs(C.row(0)[2]);
    R = abs(A0.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a1;
    R1 = b0 * abs(C.row(1)[0]) + b1 * abs(C.row(1)[1]) + b2 * abs(C.row(1)[2]);
    R = abs(A1.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a2;
    R1 = b0 * abs(C.row(2)[0]) + b1 * abs(C.row(2)[1]) + b2 * abs(C.row(2)[2]);
    R = abs(A2.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(0)[0]) + a1 * abs(C.row(1)[0]) + a2 * abs(C.row(2)[0]);
    R1 = b0;
    R = abs(B0.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(0)[1]) + a1 * abs(C.row(1)[1]) + a2 * abs(C.row(2)[1]);
    R1 = b1;
    R = abs(B1.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(0)[2]) + a1 * abs(C.row(1)[2]) + a2 * abs(C.row(2)[2]);
    R1 = b2;
    R = abs(B2.dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a1 * abs(C.row(2)[0]) + a2 * abs(C.row(1)[0]);
    R1 = b1 * abs(C.row(0)[2]) + b2 * abs(C.row(0)[1]);
    R = abs((C.row(1)[0] * A2).dot(D) - (C.row(2)[0] * A1).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a1 * abs(C.row(2)[1]) + a2 * abs(C.row(1)[1]);
    R1 = b0 * abs(C.row(0)[2]) + b2 * abs(C.row(0)[0]);
    R = abs((C.row(1)[1] * A2).dot(D) - (C.row(2)[1] * A1).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a1 * abs(C.row(2)[2]) + a2 * abs(C.row(1)[2]);
    R1 = b0 * abs(C.row(0)[1]) + b1 * abs(C.row(0)[0]);
    R = abs((C.row(1)[2] * A2).dot(D) - (C.row(2)[2] * A1).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(2)[0]) + a2 * abs(C.row(0)[0]);
    R1 = b1 * abs(C.row(1)[2]) + b2 * abs(C.row(1)[1]);
    R = abs((C.row(2)[0] * A0).dot(D) - (C.row(0)[0] * A2).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(2)[1]) + a2 * abs(C.row(0)[1]);
    R1 = b0 * abs(C.row(1)[2]) + b2 * abs(C.row(0)[1]);
    R = abs((C.row(2)[1] * A0.dot(D) - (C.row(0)[1] * A2).dot(D)));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(2)[2]) + a2 * abs(C.row(0)[2]);
    R1 = b0 * abs(C.row(1)[1]) + b1 * abs(C.row(1)[0]);
    R = abs((C.row(2)[2] * A0).dot(D) - (C.row(0)[2] * A2).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(1)[0]) + a1 * abs(C.row(0)[0]);
    R1 = b1 * abs(C.row(2)[2]) + b2 * abs(C.row(2)[1]);
    R = abs((C.row(0)[0] * A1).dot(D) - (C.row(1)[0] * A0).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(1)[1]) + a1 * abs(C.row(0)[1]);
    R1 = b0 * abs(C.row(2)[2]) + b2 * abs(C.row(2)[0]);
    R = abs((C.row(0)[1] * A1).dot(D) - (C.row(1)[1] * A0).dot(D));
    if (R > R0 + R1) { return false; }

    R0 = a0 * abs(C.row(1)[2]) + a1 * abs(C.row(0)[2]);
    R1 = b0 * abs(C.row(2)[1]) + b1 * abs(C.row(2)[0]);
    R = abs((C.row(0)[2] * A1).dot(D) - (C.row(1)[2] * A0).dot(D));
    if (R > R0 + R1) { return false; }

    return true;
}

std::shared_ptr<cg3d::Model> BasicScene::NewBox(Eigen::AlignedBox<double, 3> Frame, int  index) {
    Eigen::MatrixXd V(8, 3), VertexToNormal, C;
    Eigen::MatrixXi F(12, 3);
    Eigen::RowVector3d V0 = Frame.corner(Frame.BottomLeftFloor);
    Eigen::RowVector3d V1 = Frame.corner(Frame.TopLeftFloor);
    Eigen::RowVector3d V2 = Frame.corner(Frame.TopRightFloor);
    Eigen::RowVector3d V3 = Frame.corner(Frame.BottomRightFloor);
    Eigen::RowVector3d V4 = Frame.corner(Frame.BottomLeftCeil);
    Eigen::RowVector3d V5 = Frame.corner(Frame.TopLeftCeil);
    Eigen::RowVector3d V6 = Frame.corner(Frame.TopRightCeil);
    Eigen::RowVector3d V7 = Frame.corner(Frame.BottomRightCeil);
    V << V0[0], V0[1], V0[2], V1[0], V1[1], V1[2], V2[0], V2[1], V2[2], V3[0], V3[1],
        V3[2], V4[0], V4[1], V4[2], V5[0], V5[1], V5[2], V6[0], V6[1], V6[2], V7[0], V7[1], V7[2];
    F << 1, 2, 5, 2, 5, 6, 4, 5, 7, 5, 6, 7, 0, 3, 4, 3, 4, 7, 0, 1, 3, 1, 2, 3, 0, 1, 5, 0, 4, 5, 2, 3, 7, 2, 6, 7;
    igl::per_vertex_normals(V, F, VertexToNormal);
    C = Eigen::MatrixXd::Zero(V.rows(), 2);
    std::shared_ptr<Mesh> m = std::make_shared<Mesh>("Cube", V, F, VertexToNormal, C);
    std::vector<std::shared_ptr<Mesh>> vec;
    vec.push_back(m);
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program) };
    std::shared_ptr<cg3d::Model> model;
    if (index == 1) { model = Model::Create("cube1", m, material); }
    else if (index == 2) { model = Model::Create("cube2", m, material); }
    else { model = Model::Create("cube", m, material); }
    model->showWireframe = true;
    return model;
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

Eigen::Vector3f BasicScene::GenerateSpheres(std::shared_ptr<cg3d::Camera> camera, float a, float b) {
    std::random_device rd;
    std::mt19937 gen(rd());

    float z = std::uniform_real_distribution<>(a, b)(gen);
    float x = tan((camera->fov * (3.14 / 180)) / 2) * z * camera->ratio;
    float y = tan((camera->fov * (3.14 / 180)) / 2) * z;
    std::uniform_real_distribution<> dist1(-x, x);
    x = dist1(gen);
    std::uniform_real_distribution<> dist2(-y, y);
    y = dist2(gen);
    return camera->GetTranslation() + camera->GetRotation() * Eigen::Vector3f(x, y, -z);
}
 
/*------------------------------------------------------------Collisions in game-------------------------------------------------------------*/

void BasicScene::Collisionoftarget(int number , int Extrascore) {
    for (int i = 0; i < number; i++) {
        bool coll = Collision(&treeofCyls[cyls.size()-1], &treeofSphere[i], i, cyls.size() - 1, spehersforlevel1);
        if (coll) {
            score = score + Extrascore;
            //playsound
           // spehersforlevel1[i]->isHidden = true;
            //spehersforlevel1[i]->Translate(-(initPosSphere[i]));
            //spehersforlevel1[i]->SetCenter({ 0,0,0 });

            auto newVec = GenerateSpheres(cameraslist[0], 5, 70);
            //initPosSphere[i] = newVec;

            spehersforlevel1[i]->Translate((newVec - (spehersforlevel1[i]->GetTranslation())).normalized() * 10);
            pickedModel = nullptr;

           // spehersforlevel1[i]->isHidden = false;
        }
    }
}
void BasicScene::CollisionoftargetHard(int number , int Extrascore) {
    for (int i = 0; i < number; i++) {
        bool coll = Collision(&treeofCyls[cyls.size()-1], &treeofSphere[i], i, cyls.size() - 1, spehersforlevel1);
        if (coll) {
            score = score + Extrascore;
            auto newVec = GenerateSpheres(cameraslist[0], 5, 70);
            spehersforlevel1[i]->Translate((newVec - (spehersforlevel1[i]->GetTranslation())).normalized() * 10);
            spehersforlevel1[indexofColl + 1]->isHidden = false;
            indexofColl++;
            pickedModel = nullptr;

        }
    }
}

void BasicScene::Collisionofbombers(int number) {
    for (int i = 0; i < number; i++) {
        bool coll = Collision(&treeofCyls[cyls.size() - 1], &treeofbombers[i], i, cyls.size() - 1, bombers);
        if (coll) {
            score = 0;
           // bombers[i]->Translate(-(initPosBomber[i]));
            auto newVec = GenerateSpheres(cameraslist[0], 5, 70);
            //initPosBomber[i] = newVec;
            //playsound
             state = Statue::Looser;
             pickedModel = nullptr;
        }
    }
}


void BasicScene::Collisionofgift(int number, int Extrascore) {
    for (int i = 0; i < number; i++) {
        bool coll = Collision(&treeofCyls[cyls.size() - 1], &treeofgifts[i], i, cyls.size() - 1,gifts);
        if (coll) {
            score = score + Extrascore;
            //playsound
           // gifts[i]->isHidden = true;

           // gifts[i]->Translate(-(initPosGift[i]));
            auto newVec = GenerateSpheres(cameraslist[0], 5, 70);
            gifts[i]->Translate((newVec - (gifts[i]->GetTranslation())).normalized() * 10);
           // initPosGift[i] = newVec;
            pickedModel = nullptr;

           // gifts[i]->isHidden = false;
           // speed = speed + 0.05;
        }
    }
}
/*------------------------------------------------------------End of Collisions in game-------------------------------------------------------------*/
/*------------------------------------------------------------reset Models in game-------------------------------------------------------------*/

void BasicScene::resetSpheres(int number) {
    for (int i = 0; i < spehersforlevel1.size(); i++) {
        
        if (i < number) {
           // spehersforlevel1[i]->isHidden = true;
            if(num == 1){
            auto newVec = GenerateSpheres(cameraslist[0], 10, 75);
            //initPosSphere.push_back(newVec);
            spehersforlevel1[i]->Translate(newVec);
            }
            else {
                
             spehersforlevel1[i]->Translate({i+0.1f/10,i+0.1f/10,i+0.1f/10});
            }
            //spehersforlevel1[i]->isHidden = false;
        }
        else {
            spehersforlevel1[i]->isHidden = true;
        }
    }
    
}


void BasicScene::resetgifts(int number) {
    if (CurrentLevel == 4 || CurrentLevel == 5 || CurrentLevel == 6) {
        speed = speed + 0.001;
    }
    for (int i = 0; i < gifts.size(); i++) {
        if (i < number) {
            //gifts[i]->isHidden = true;
            
            if (num == 1) {
                auto newVec = GenerateSpheres(cameraslist[0], 10, 75);
                //initPosGift.push_back(newVec);
                gifts[i]->Translate(newVec);
                //gifts[i]->isHidden = false;
            }
            else {
                gifts[i]->Translate({ i + 0.1f / 10,i + 0.1f / 10,i + 0.1f / 10 });

            }
            
        }
        else {
            gifts[i]->isHidden = true;
        }
    }
}

void BasicScene::resetbombers(int number) {
    for (int i = 0; i < bombers.size(); i++) {
        if (i < number) {
            if (num == 1) {
                // bombers[i]->isHidden = true;
                auto newVec = GenerateSpheres(cameraslist[0], 10, 75);
                // initPosBomber.push_back(newVec);
                bombers[i]->Translate(newVec);
                //bombers[i]->isHidden = false;
            }
            else {
                bombers[i]->Translate({ i + 0.1f / 10,i + 0.1f / 10,i + 0.1f / 10 });

            }
        }
        else {
            bombers[i]->isHidden = true;
        }
    }
}
/*------------------------------------------------------------ End of reset Models in game-------------------------------------------------------------*/
/*------------------------------------------------------------ game-------------------------------------------------------------*/

void BasicScene::EasyGameLevel1() {
    WK = false;
    QK = false;
    EK = false;
    AK = false;
    SK = false;
    DK = false;
    speed = 0.01f;
    easyGame = true;
    HardGame = false;
    paused = false;
    score = 0;
    resetSpheres(10);
    resetgifts(1);
    
    Time = std::chrono::steady_clock::now();
}
void BasicScene::EasyGameLevel2() {

    WK = false;
    QK = false;
    EK = false;
    AK = false;
    SK = false;
    DK = false;
    speed = 0.01f;

    easyGame = true;
    HardGame = false;
    paused = false;
    score = 0;
    resetSpheres(5);
    resetgifts(1);
    Time = std::chrono::steady_clock::now();
}
void BasicScene::EasyGameLevel3() {
    WK = false;
    QK = false;
    EK = false;
    AK = false;
    SK = false;
    DK = false;
    speed = 0.01f;

    easyGame = true;
    HardGame = false;
    paused = false;
    score = 0;

    resetSpheres(3);
    resetgifts(1);
    resetbombers(2);
    Time = std::chrono::steady_clock::now();
}
void BasicScene::HardGameLevel1() {
    WK = false;
    QK = false;
    EK = false;
    AK = false;
    SK = false;
    DK = false;
    speed = 0.01f;

    easyGame = false;
    HardGame = true;
    paused = false;
    score = 0;
    resetSpheres(1);
    resetgifts(2);
    Time = std::chrono::steady_clock::now();
}
void BasicScene::HardGameLevel2() {
    WK = false;
    QK = false;
    EK = false;
    AK = false;
    SK = false;
    DK = false;
    speed = 0.01f;

    easyGame = false;
    HardGame = true;
    paused = false;
    score = 0;
    resetSpheres(1);
    Time = std::chrono::steady_clock::now();
}
void BasicScene::HardGameLevel3() {
    WK = false;
    QK = false;
    EK = false;
    AK = false;
    SK = false;
    DK = false;
    speed = 0.01f;
    indexofColl = 0;
    easyGame = false;
    HardGame = true;
    paused = false;
    score = 0;
    resetSpheres(1);
    resetgifts(1);
    resetbombers(2);
    Time = std::chrono::steady_clock::now();
}

void BasicScene::Game(int level)
{
    selfColl = false;
    winner = false;
    if (train) {
        state = Statue::inTrain;
    }else{
    state = Statue::InGame;
    }
    if (easyGame) {
        if (level == 1) {
            EasyGameLevel1();
        }
        else if (level == 2) {
            EasyGameLevel2();
        }
        else if (level == 3) {
            EasyGameLevel3();
        }
    }
    else if (HardGame) {
        if (level == 1) {
            HardGameLevel1();
        }
        else if (level == 2) {
            HardGameLevel2();
        }
        else if (level == 3) {
            HardGameLevel3();
        }
    }
}
/*------------------------------------------------------------ End of game-------------------------------------------------------------*/
/*------------------------------------------------------------ ImGui-------------------------------------------------------------*/
Eigen::MatrixXd BasicScene::scaled(const Eigen::MatrixXd& vertices, const Eigen::Vector3d& scale)
{
    Eigen::DiagonalMatrix<double, 3> scalingMatrix(scale);

    Eigen::MatrixXd scaledVertices = vertices * scalingMatrix;

    return scaledVertices;
}
void BasicScene::curs_pos(std::string text, float margin )
{
    ImGui::SetCursorPosX((ImGui::GetWindowSize().x - ImGui::CalcTextSize(text.c_str()).x) * 0.5f);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + margin);
}
void BasicScene::text_pos(std::string text, float margin )
{
    curs_pos(text, margin);
    ImGui::Text(text.c_str());
}

void BasicScene::BuildImGui(){
    int flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    bool* pOpen = nullptr;
    if (state == Statue::InGame || state == Statue::inTrain) {
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiCond_Always);
    }
    else {
        ImVec2 disSize = ImGui::GetIO().DisplaySize;
        ImVec2 windowSize(400, 300);
        ImVec2 windowPos(disSize.x / 2 - windowSize.x / 2, disSize.y / 2 - windowSize.y / 2);
        ImGui::SetNextWindowSize(windowSize);
        ImGui::SetNextWindowPos(windowPos);
    }
    ImGui::Begin("Menu", pOpen, flags);
   
    switch (state)
    {
    case BasicScene::start: {
        text_pos("Welcome To Snake Game",10);
        curs_pos("Training", 30);
        if (ImGui::Button("Training"))
        {
            
            state = Statue::training;
             
             //PlaySound("data/bonus-earned-in-video-game-2058.wav", NULL, SND_FILENAME | SND_ASYNC);
        }
        curs_pos("Start Game",30);
        if (ImGui::Button("Start Game"))
        {
            state = Statue::KindofGame;
        }

        curs_pos("Quit Game", 30);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case BasicScene::KindofGame: {
        text_pos("Choose the difficulty of the game", 10);
        curs_pos("Easy Game", 50);
        if (ImGui::Button("Easy Game"))
        {
            easyGame = true;
            HardGame = false;
            state = Statue::LevelofEasyGame;
        } 
        curs_pos("Hard Game", 30);
        if(ImGui::Button("Hard Game"))
        {
            easyGame = false;
            HardGame = true;
            state = Statue::LevelofHardGame;
        }
        curs_pos("Quit Game", 30);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case BasicScene::LevelofEasyGame: {
        text_pos("Choose the level of the game", 10);
        curs_pos("Level 1", 50);
        if (ImGui::Button("Level 1")) {
            state = Statue::EasyGamelevel1;
        }
        curs_pos("Level 2", 30); 
        if (ImGui::Button("Level 2")) {
            state = Statue::EasyGamelevel2;
        }
        curs_pos("Level 3", 30);
        if (ImGui::Button("Level 3")) {
            state = Statue::EasyGamelevel3;
        }
        curs_pos("Quit Game", 30);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case BasicScene::LevelofHardGame: {
        text_pos("Choose the level of the game", 10);
        curs_pos("Level 1", 30);
        if (ImGui::Button("Level 1")) {
            state = Statue::HardGamelevel1;
        }
        curs_pos("Level 2", 30);
        if (ImGui::Button("Level 2")) {
            state = Statue::HardGamelevel2;
        }
        curs_pos("Level 3", 30);
        if (ImGui::Button("Level 3")) {
            state = Statue::HardGamelevel3;
        }
        curs_pos("Quit Game", 30);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case BasicScene::training: {
        Timeofgame = std::chrono::seconds(10000);
        train = true;
        easyGame = true;
        
        CurrentLevel = 1;
        text_pos("Training:- ", 10);
        text_pos("Its to learn about the controls in the game", 10);
        text_pos("Contrlos:-", 20);
        text_pos("Space:-   To move  ", 10);
        text_pos("1:-   first camera(Static camera)  ", 10);
        text_pos("2:-   second camera(from head the snake)  ", 10);
        text_pos("W:-   To move to up of short Z", 10);
        text_pos("S:-   To move to down of short Z", 10);
        text_pos("D:-   To move to right of short X", 10);
        text_pos("A:-   To move to left of short X", 10);
        text_pos("Q:-   To move to front of short Y", 10);
        text_pos("E:-   To move to back of short Y", 10);
        curs_pos("Start the train", 10);
        if (ImGui::Button("Start the train")) {
            num++;
            playnow = true;
            Game(1);
        }
        break;
    }
    case BasicScene::EasyGamelevel1: {
        Timeofgame = std::chrono::seconds(10000);
        CurrentLevel = 1;
        text_pos("Level 1 of easy game", 10);
        text_pos("You need to earn 80 points to win", 10);
        text_pos("Eat the spheres, each sphere give you 10 point", 10);
        text_pos("Eat the ball, each ball give you 15 point", 10);
        text_pos("You must pick the modle that you need to move to them and pick space to move", 10);
        text_pos("You have 1 minute", 10);
        text_pos("Controls:-", 20);
        text_pos("Space:-   To move  ", 10);
        text_pos("1:-   first camera(Static camera)  ", 10);
        text_pos("2:-   second camera(from head the snake)  ", 10);
        curs_pos("Start", 10);
        if (ImGui::Button("start")) {
            num++;
            Game(1);
        }
        break;
    }
    case BasicScene::EasyGamelevel2: {
        Timeofgame = std::chrono::seconds(40);
        CurrentLevel = 2;
        text_pos("Level 2 of easy game", 10);
        text_pos("You need to earn 120 points to win", 10);
        text_pos("Eat the spheres, each sphere give you 10 point", 10);
        text_pos("Eat the ball, each ball give you 15 point", 10);
        text_pos("You must pick the modle that you need to move to them and pick space to move", 10);
        text_pos("You have 40 Seconds", 10);
        text_pos("Controls:-", 20);
        text_pos("Space:-   To move  ", 10);
        text_pos("1:-   first camera(Static camera)  ", 10);
        text_pos("2:-   second camera(from head the snake)  ", 10);
        curs_pos("Start", 10);
        if (ImGui::Button("start")) {
            num++;
            Game(2);
        }
        break;
    }
    case BasicScene::EasyGamelevel3: {
        Timeofgame = std::chrono::seconds(40);
        CurrentLevel = 3;
        text_pos("Level 3 of easy game", 10);
        text_pos("You need to earn 120 points to win", 10);
        text_pos("Eat the spheres, each sphere give you 10 point", 10);
        text_pos("Eat the ball, each ball give you 15 point", 10);
        text_pos("Avoid the cube, if you eat the cube you will lose", 10);
        text_pos("You must pick the modle that you need to move to them and pick space to move", 10);
        text_pos("You have 40 Seconds", 10);
        text_pos("Controls:-", 20);
        text_pos("Space:-   To move  ", 10);
        text_pos("1:-   first camera(Static camera)  ", 10);
        text_pos("2:-   second camera(from head the snake)  ", 10);
        curs_pos("Start", 10);
        if (ImGui::Button("start")) {
            num++;
            Game(3);
        }
        break;
    }
    case BasicScene::HardGamelevel1: {
        Timeofgame = std::chrono::seconds(1000);
        CurrentLevel = 1;
        text_pos("Level 1 of Hard game", 10);
        text_pos("You need to earn 50 points to win", 10);
        text_pos("Eat the spheres, each sphere give you 10 point", 10);
        text_pos("Eat the ball, each ball give you 15 point and will give you a extra speed ", 10);
        text_pos("You must move with the keyboard to your targets", 10);
        text_pos("You have 1 minute", 10);
        text_pos("Controls:-", 20);
        text_pos("W:-   To move to up of short Z", 10);
        text_pos("S:-   To move to down of short Z", 10);
        text_pos("D:-   To move to right of short X", 10);
        text_pos("A:-   To move to left of short X", 10);
        text_pos("Q:-   To move to front of short Y", 10);
        text_pos("E:-   To move to back of short Y", 10);
        text_pos("1:-   first camera(Static camera)  ", 10);
        text_pos("2:-   second camera(from head the snake)  ", 10);
        curs_pos("Start", 10);
        if (ImGui::Button("start")) {
            num++;
            Game(1);
        }
        break;
    }
    case BasicScene::HardGamelevel2: {
        Timeofgame = std::chrono::seconds(1000);
        CurrentLevel = 2;

        text_pos("Level 2 of Hard game", 10);
        text_pos("You need to earn 80 points to win", 10);
        text_pos("Eat the spheres, each sphere give you 10 point", 10);
        text_pos("Eat the ball, each ball give you 15 point ", 10);
        text_pos("and will give you a extra speed", 10);
        text_pos("You must move with the keyboard to your targets", 10);
        text_pos("You have 50 Seconds", 10);
        text_pos("Controls:-", 20);
        text_pos("W:-   To move to up of short Z", 10);
        text_pos("S:-   To move to down of short Z", 10);
        text_pos("D:-   To move to right of short X", 10);
        text_pos("A:-   To move to left of short X", 10);
        text_pos("Q:-   To move to front of short Y", 10);
        text_pos("E:-   To move to back of short Y", 10);
        text_pos("1:-   first camera(Static camera)  ", 10);
        text_pos("2:-   second camera(from head the snake)  ", 10);
        curs_pos("Start", 10);
        if (ImGui::Button("start")) {
            num++;
            Game(2);
        }
        break;
    }
    case BasicScene::HardGamelevel3: {
        Timeofgame = std::chrono::seconds(45);
        CurrentLevel = 3;
        text_pos("Level 3 of Hard game", 10);
        text_pos("You need to earn 100 points to win", 10);
        text_pos("Eat the spheres, each sphere give you 10 point", 10);
        text_pos("Eat the ball, each ball give you 15 point and will give you a extra speed ", 10);
        text_pos("Avoid the cube, if you eat the cube you will lose", 10);
        text_pos("You must move with the keyboard to your targets", 10);
        text_pos("You have 45 Seconds", 10);
        text_pos("Controls:-", 20);
        text_pos("W:-   To move to up of short Z", 10);
        text_pos("S:-   To move to down of short Z", 10);
        text_pos("D:-   To move to right of short X", 10);
        text_pos("A:-   To move to left of short X", 10);
        text_pos("Q:-   To move to front of short Y", 10);
        text_pos("E:-   To move to back of short Y", 10);
        text_pos("1:-   first camera(Static camera)  ", 10);
        text_pos("2:-   second camera(from head the snake)  ", 10);
        curs_pos("Start", 10);
        if (ImGui::Button("start")) {
            num++;
            Game(3);
        }
        break;
    }
    case BasicScene::InGame: {

       ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiCond_Always);
        playnow = true;
        text_pos(("Score: " + std::to_string(score)), 0);
        remainning = std::chrono::duration_cast<std::chrono::seconds>(Timeofgame - (std::chrono::steady_clock::now() - Time)).count();
        text_pos(("Timer: " + std::to_string(remainning)), 10);
        curs_pos("Pause", 10);
        if (ImGui::Button("Pause")) {
            state = Statue::pause;
        }
        break;
    }case BasicScene::inTrain: {
        curs_pos("leave The Train", 10);
        if (ImGui::Button("leave The Train")) {
            RestartTheSnakeToInit();
            train = false;
            playnow = false;
            easyGame = false;
            speed = 0.01f;
            WK = false;
            QK = false;
            EK = false;
            AK = false;
            SK = false;
            DK = false;
            score = 0;
            state = Statue::start;
        }
        break;
    }
    case BasicScene::Looser: {
        playnow = false;
        std::cout << "Score The Looser Is: " << std::to_string(score) << std::endl;
        text_pos("Game Over", 0);
        text_pos(("Score: " + std::to_string(score)), 0);
        curs_pos("try again", 10);
        if (ImGui::Button("try again")) {
            RestartTheSnakeToInit();
            if (easyGame && CurrentLevel == 1) {
                state = Statue::EasyGamelevel1;
            }
            if (easyGame && CurrentLevel == 2) {
                state = Statue::EasyGamelevel2;
            }
            if (easyGame && CurrentLevel == 3) {
                state = Statue::EasyGamelevel3;
            }
            if (HardGame && CurrentLevel == 1) {
                state = Statue::HardGamelevel1;
            }
            if (HardGame && CurrentLevel == 2) {
                state = Statue::HardGamelevel2;
                
            }
            if (HardGame && CurrentLevel == 3) {
                state = Statue::HardGamelevel3;
            }
        }
        curs_pos("Menu", 10);
        if (ImGui::Button("Menu"))
        {
            RestartTheSnakeToInit();
            train = false;
            playnow = false;
            easyGame = false;
            speed = 0.01f;
            WK = false;
            QK = false;
            EK = false;
            AK = false;
            SK = false;
            DK = false;
            score = 0;
            state = Statue::start;
        }
        curs_pos("Quit Game", 10);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case BasicScene::Winner: {
        playnow = false;
        std::cout << "Score The Winner Is: " << std::to_string(score) << std::endl;
        text_pos("Congratulation", 0);
        text_pos(("Score The Winner Is: " + std::to_string(score)), 0);
        curs_pos("Go To Next Level", 10);
        if (ImGui::Button("Go To Next Level")) {
            RestartTheSnakeToInit();
                if (easyGame && CurrentLevel == 1) {
                    state = Statue::EasyGamelevel2;
                }
                if (easyGame && CurrentLevel == 2) {
                    state = Statue::EasyGamelevel3;
                }
                if (easyGame && CurrentLevel == 3) {
                    state = Statue::HardGamelevel1;
                }
                if (HardGame && CurrentLevel == 1) {
                    state = Statue::HardGamelevel2;
                }
                if (HardGame && CurrentLevel == 2) {
                    state = Statue::HardGamelevel3;
                }
                if (HardGame && CurrentLevel == 3) {
                    curs_pos("Quit Game", 10);
                    if (ImGui::Button("Quit Game"))
                    {
                        exit(0);
                    }
                }
            
        }
        curs_pos("Menu", 10);
        if (ImGui::Button("Menu"))
        {
            RestartTheSnakeToInit();
            train = false;
            playnow = false;
            easyGame = false;
            speed = 0.01f;
            WK = false;
            QK = false;
            EK = false;
            AK = false;
            SK = false;
            DK = false;
            score = 0;
            state = Statue::start;
        }
        curs_pos("Quit Game", 10);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    case BasicScene::pause: {
        playnow = false;
        std::cout << "Score: " << std::to_string(score) << std::endl;
        text_pos("Paused", 20);
        text_pos(("Your score in the game is: " + std::to_string(score)), 20);
        curs_pos("continue", 10);
        if (ImGui::Button("continue")) {
            state = Statue::InGame;
        }
        curs_pos("Restart", 10);
        if (ImGui::Button("Restart")) {
            RestartTheSnakeToInit();
            if (easyGame && CurrentLevel == 1) {
                state = Statue::EasyGamelevel1;
            }
            if (easyGame && CurrentLevel == 2) {
                state = Statue::EasyGamelevel2;
            }
            if (easyGame && CurrentLevel == 3) {
                state = Statue::EasyGamelevel3;
            }
            if (HardGame && CurrentLevel == 1) {
                state = Statue::HardGamelevel1;
            }
            if (HardGame && CurrentLevel == 2) {
                state = Statue::HardGamelevel2;
            }
            if (HardGame && CurrentLevel == 3) {
                state = Statue::HardGamelevel3;
            }
        }
        curs_pos("Quit Game", 10);
        if (ImGui::Button("Quit Game"))
        {
            exit(0);
        }
        break;
    }
    default: {
        text_pos("Error", 30);
        break;
    }
    }
    ImGui::End();
}
/*---------------------------------------------------------------------------Self collision---------------------------------------------------------------------------*/

void BasicScene::SelfCollision() {
    for (int i = 0; i < cyls.size()-2 && !selfColl; i++) {
        bool coll = Collision(&treeofCyls[cyls.size() - 1], &treeofCyls[i], i, cyls.size() - 1, cyls);
        if (coll) {
            std::cout << "Game Over" << std::endl;
            selfColl = true;
            state = Statue::Looser;
        }
    }
}

/*-------------------------------------------------------------skinning---------------------------------------------------------------------------------------------------*/


void BasicScene::Weights(Eigen::Vector3f posV, Eigen::MatrixXd C,int j)
{
    std::vector<float> dist(C.rows()); 
    for (int i = 0; i < C.rows(); i++){
        Eigen::Vector3f posC_float = C.row(i).cast<float>().eval();
        dist[i] = (posV - posC_float).norm(); 
    }

     float min = dist[0];
    int minindex1 = 0;
    for (int i = 1; i < C.rows(); i++) {
        if (dist[i] <= min) {
            min = dist[i];
            minindex1 = i;
            dist[i] = INFINITY;
        }
    }

    float min2 = dist[0];
    int minindex2 = 0;
        for (int i = 1; i < C.rows(); i++) {
            if (dist[i] <= min2) {
                std::cout << "I :" << i << std::endl;

                min2 = dist[i];
                minindex2 = i;
            }
        }
        if (minindex1 == C.rows() -1)
        {
    
            minindex1 = minindex2;
        }
        else if (minindex2 == C.rows() - 1)
        {
            minindex2 = minindex1;
        }

     Eigen::Vector2f weights = Eigen::Vector2f(-1 / (powf(min, 2.0f) + 1), -1 / (powf(min2, 2.0f) + 1)).normalized();
     std::cout << minindex1 << std::endl;
     std::cout << "BBBBB"<<minindex2 << std::endl;
     std::cout <<"Hiii" <<W.row(j).cols() << std::endl;
    W.row(j)[minindex1] = weights[0];
    W.row(j)[minindex2] = weights[1];
    
}


void BasicScene::skinning()
{
    RotationList Currpos(cyls.size());
    RotationList vQ;
    std::vector<Eigen::Vector3d> vT;
    for (size_t i = 0; i < cyls.size(); i++)
    {
        Eigen::Quaternionf Qrot = Eigen::Quaternionf(cyls[i]->GetTout().rotation()).normalized();
        Currpos[i] = Eigen::Quaterniond(Qrot.w(), Qrot.x(), Qrot.y(), Qrot.z());
    }
    igl::forward_kinematics(C, BE, P, Currpos, vQ, vT);//take c->joint position,BE->matrix of bone pos , p -> matrix of parent bone , anim->
    igl::dqs(V, W, vQ, vT, U);
    Eigen::MatrixXd normals;
    igl::per_vertex_normals(U, snake->GetMesh()->data[0].faces, normals);
    Mesh nextPose("snake", U, F, normals, snake->GetMesh()->data[0].textureCoords);
    snake->SetMeshList({ std::make_shared<Mesh>(nextPose) });
    
}

