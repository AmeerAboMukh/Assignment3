#pragma once

#include "AutoMorphingModel.h"
#include "Scene.h"
#include "imgui.h"
#include "igl/AABB.h"
#include "SceneWithImGui.h"
#include "file_dialog_open.h"
#include <memory>
#include <utility>
#include <ctime>
#include <ratio>
#include <chrono>
class BasicScene : public cg3d::SceneWithImGui
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : SceneWithImGui(std::move(name), display) {
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.FrameRounding = 5.0f;
    };
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    Eigen::Vector3f GetSpherePos();
    Eigen::Vector3f GetTipPosOrGetSourcePos(int cylIndex, int mod);
    void OurRotate(int mod, float number1, float number2);
    void CCDMethod();
    bool Collision_Conditions(Eigen::AlignedBox<double, 3> Frame1, Eigen::AlignedBox<double, 3> Frame2, int index1, int index2, std::vector<std::shared_ptr<cg3d::Model>> x);
    bool Collision(igl::AABB <Eigen::MatrixXd, 3>* tree1, igl::AABB <Eigen::MatrixXd, 3>* tree2, int index1, int index2, std::vector<std::shared_ptr<cg3d::Model>> x);
    void Collisionoftarget(int number,int Extrascore);
    void Collisionofbombers(int number);
    void Collisionofgift(int number , int Extrascore);
    std::shared_ptr<cg3d::Model> NewBox(Eigen::AlignedBox<double, 3> Frame, int index);
    void FabrikMethodEasyGame(Eigen::Vector3f target,float x);
    void FabrikMethodHardGame(Eigen::Vector3f target);
    void BuildImGui() override;
    void resetSpheres(int number);
    void resetgifts(int number);
    void resetbombers(int number);
    void EasyGameLevel1();
    void EasyGameLevel2();
    void EasyGameLevel3();
    void HardGameLevel1();
    void HardGameLevel2();
    void HardGameLevel3();
    void Game(int lev);
    void text_pos(std::string text, float margin);
    void curs_pos(std::string text,  float margin);
    void SelfCollision();
    void RestartTheSnakeToInit();
    void CollisionoftargetHard(int number, int Extrascore);
    /*******/
    Eigen::MatrixXd scaled(const Eigen::MatrixXd& vertices, const Eigen::Vector3d& scale);
    void skinning();
    void Weights(Eigen::Vector3f posV, Eigen::MatrixXd C,int i);

private:
    Eigen::Vector3f GenerateSpheres(std::shared_ptr<cg3d::Camera> camera, float a, float b);
    std::shared_ptr<Movable> root;

    std::shared_ptr<cg3d::Model> sphere1, cube,snake,gift,bomber,optCyl,grace1;
    std::shared_ptr<cg3d::AutoMorphingModel> autoCube;
    std::vector<std::shared_ptr<cg3d::Model>> cyls, axis , spehersforlevel1,bombers,gifts;
    std::vector<std::shared_ptr<cg3d::Camera>> cameraslist;
    std::vector<igl::AABB <Eigen::MatrixXd, 3>> treeofSphere,treeofCyls,treeofbombers,treeofgifts;
    std::vector<std::shared_ptr<cg3d::Model>> BoxofSphere,BoxofCyls;
    std::shared_ptr<cg3d::Model> cyl;
    int pickedIndex = 0;
    int tipIndex = 0;
    Eigen::Vector3f half_length = Eigen::Vector3f(0, 0, 0.8f);
    std::vector<Eigen::Vector3f> initPosSphere, initPosGift, initPosBomber;
    Eigen::VectorXi EMAP,P;
    Eigen::MatrixXi F, E, EF, EI,BE;
    Eigen::VectorXi EQ;
    // If an edge were collapsed, we'd collapse it to these points:
    Eigen::MatrixXd V, C, N, T, points, edges, colors,U,W;

    igl::AABB <Eigen::MatrixXd, 3> bunny1tree, bunny2tree, bunny3tree;
    enum Statue {
        start,
        training,
        inTrain,
        KindofGame,
        LevelofEasyGame,
        EasyGamelevel1,
        EasyGamelevel2,
        EasyGamelevel3,
        HardGamelevel1,
        HardGamelevel2,
        HardGamelevel3,
        LevelofHardGame,
        InGame,
        Looser,
        Winner,
        pause,

    };
    bool easyGame = false;
    bool HardGame = false;
    bool animation = true;

    Eigen::Vector3f source;
    float speed = 1.f;
    bool stat = true;
    bool paused = true;
    int score = 0;
    std::chrono::steady_clock::time_point Time;
    std::chrono::seconds Timeofgame;
    Statue state = Statue::start;
    int CurrentLevel;
    int DeterScore1 = 50, DeterScore2 = 120, DeterScore3 = 120, DeterScore4 = 50, DeterScore5 = 80, DeterScore6 = 100;
    std::chrono::seconds::rep remainning{1000};
    bool train = false;
    bool winner = false;
    bool selfColl = false;
    bool playnow = false;
    /***************/
    typedef
        std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
        RotationList;
    int linksSize = 8;
    bool WK = false, QK = false, EK = false, AK = false, SK = false, DK = false, stop = true;;
    int iD =0,iA = 0;
    int jW=0,jS = 0;
    float fov1, near1, far1;
    int width1, height1;
    int num = 0;
    int indexofColl = 0;
};