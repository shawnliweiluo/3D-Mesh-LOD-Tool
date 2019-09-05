/*
    Sample code by Wallace Lira <http://www.sfu.ca/~wpintoli/> based on
    the four Nanogui examples and also on the sample code provided in
          https://github.com/darrenmothersele/nanogui-test
    
    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/progressbar.h>
#include <nanogui/entypo.h>
#include <nanogui/messagedialog.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/colorwheel.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>
#include <nanogui/glcanvas.h>
#include <iostream>
#include <string>

// Includes for the GLTexture class.
#include <cstdint>
#include <memory>
#include <utility>

#include "wingededge.h"


#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::pair;
using std::to_string;

using nanogui::Screen;
using nanogui::Window;
using nanogui::GroupLayout;
using nanogui::Button;
using nanogui::CheckBox;
using nanogui::Vector2f;
using nanogui::Vector2i;
using nanogui::MatrixXu;
using nanogui::MatrixXf;
using nanogui::Label;
using nanogui::Arcball;


class MyGLCanvas : public nanogui::GLCanvas {
public:
    MyGLCanvas(Widget *parent) : nanogui::GLCanvas(parent) {
    using namespace nanogui;

	mShader.initFromFiles("a_smooth_shader", "StandardShading.vertexshader", "StandardShading.fragmentshader");
    mArcball.setSize({400,400});
    
    mDrawFaces = true;
    mDrawLines = false;

	// After binding the shader to the current context we can send data to opengl that will be handled
	// by the vertex shader and then by the fragment shader, in that order.
	// if you want to know more about modern opengl pipeline take a look at this link
	// https://www.khronos.org/opengl/wiki/Rendering_Pipeline_Overview
    mShader.bind();

    mShader.uploadAttrib("vertexPosition_modelspace", positions);
    mShader.uploadAttrib("color", colors);
	mShader.uploadAttrib("vertexNormal_modelspace", normals);
    
    //ProjectionMatrixID
    float fovy = 90.0f, aspect = 1.0f;
    float near = 0.1f, far = 100.0f;
    float top = near * tan(fovy * M_PI / 360);
    float bottom = -top;
    float right = top * aspect;
    float left = -right;
    Matrix4f P = frustum(left, right, bottom, top, near, far);
    
    mShader.setUniform("P", P);

	// ViewMatrixID
	// change your rotation to work on the camera instead of rotating the entire world with the MVP matrix
    Matrix4f V = lookAt(Vector3f(0,0,4), Vector3f(0,0,0), Vector3f(0,1,0));
    mShader.setUniform("V", V);

    mTranslation = Vector3f(0, 0, 0);
    mScale = Vector3f(1, 1, 1);

	// This the light origin position in your environment, which is totally arbitrary
	// however it is better if it is behind the observer
	mShader.setUniform("LightPosition_worldspace", Vector3f(-2,6,4));

    }

    //flush data on call
    ~MyGLCanvas() {
        mShader.free();
    }

    //Method to update the rotation on each axis
    void setRotation(nanogui::Matrix4f matRotation) {
        mRotation = matRotation;
    }

    void setScale(nanogui::Vector3f vScale) {
        mScale = vScale;
    }

    void setTranslation(nanogui::Vector3f vTranslation) {
        mTranslation = vTranslation;
    }

    //Method to update the mesh itself, can change the size of it dynamically, as shown later
    void updateMeshPositions(MatrixXf newPositions){
        positions = newPositions;
    }

    void updateMeshNormals(MatrixXf newNormals){
        normals = newNormals;
    }

    void updateMeshColors(MatrixXf newColors){
        colors = newColors;
    }

    void setDrawFaces(bool drawFaces)
    {
        mDrawFaces = drawFaces;
    }

    void setDrawLines(bool drawLines)
    {
        mDrawLines = drawLines;
    }

    virtual bool mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) override {
        if (button == GLFW_MOUSE_BUTTON_2) {    // right click
            mArcball.button(p, down);
            return true;
        }
        return false;
    }

    virtual bool mouseMotionEvent(const Eigen::Vector2i &p, const Vector2i &rel, int button, int modifiers) override {
        if (button == GLFW_MOUSE_BUTTON_3 ) {   // right click
            mArcball.motion(p);
            return true;
        }
        return false;
    }

    virtual bool mouseDragEvent(const Vector2i &p, const Vector2i &rel, int button, int modifiers) override {
        if (button == GLFW_MOUSE_BUTTON_2) {    // left click
            setTranslation(mTranslation + Eigen::Vector3f(rel.x()/80.0f, -rel.y()/80.0f, 0.0f));
            return true;
        }
        return false;
    }

    virtual bool scrollEvent(const Vector2i &p, const Vector2f &rel) override {
        float scaleFactor = rel.y()/4.0f;
        if(mScale.x() + scaleFactor <= 0 || mScale.y() + scaleFactor <= 0 || mScale.z() + scaleFactor <= 0)
        {
            return false;
        }
        setScale(mScale + Eigen::Vector3f(scaleFactor, scaleFactor, scaleFactor));
        return true;
    }

    //OpenGL calls this method constantly to update the screen.
    virtual void drawGL() override {
        using namespace nanogui;

	    //refer to the previous explanation of mShader.bind();
        mShader.bind();

	    //this simple command updates the positions matrix. You need to do the same for color and indices matrices too
	    mShader.uploadAttrib("vertexPosition_modelspace", positions);
        mShader.uploadAttrib("color", colors);
	    mShader.uploadAttrib("vertexNormal_modelspace", normals);
        
        //ModelMatrixID
        setRotation(mArcball.matrix());
        Matrix4f M = translate(mTranslation) * mRotation * scale(mScale);
        mShader.setUniform("M", M);

	    // If enabled, does depth comparisons and update the depth buffer.
	    // Avoid changing if you are unsure of what this means.
        glEnable(GL_DEPTH_TEST);
	
        /* Draw 12 triangles starting at index 0 of your indices matrix */
	    /* Try changing the first input with GL_LINES, this will be useful in the assignment */
	    /* Take a look at this link to better understand OpenGL primitives */
	    /* https://www.khronos.org/opengl/wiki/Primitive */
        

        // each face outputs 3 vertices and 3 edges(lines), which are 6 additional vertices, into positions
        if(mDrawFaces)
        {
	        mShader.drawArray(GL_TRIANGLES, 0, positions.cols() / 3);
        }

        if(mDrawLines)
        {
	        mShader.drawArray(GL_LINES, positions.cols() / 3, positions.cols());
        }        

        glDisable(GL_DEPTH_TEST);
    }

//Instantiation of the variables that can be acessed outside of this class to interact with the interface
//Need to be updated if a interface element is interacting with something that is inside the scope of MyGLCanvas
private:
    MatrixXf positions;
	MatrixXf normals;
    MatrixXf colors;
    nanogui::GLShader mShader;
    Eigen::Matrix4f mRotation;
    Eigen::Vector3f mScale;
    Eigen::Vector3f mTranslation;
    nanogui::Arcball mArcball;
    bool mDrawFaces;
    bool mDrawLines;
};


class ExampleApplication : public nanogui::Screen {
public:
    ExampleApplication() : nanogui::Screen(Eigen::Vector2i(850, 680), "NanoGUI Cube and Menus", false) {
        using namespace nanogui;

    //First, we need to create a window context in which we will render both the interface and OpenGL canvas
    Window *window = new Window(this, "GLCanvas");
    window->setPosition(Vector2i(15, 15));
    window->setLayout(new GroupLayout());

    //OpenGL canvas initialization, we can control the background color and also its size
    mCanvas = new MyGLCanvas(window);
    mCanvas->setBackgroundColor({100, 100, 100, 255});
    mCanvas->setSize({400, 400});
    mMesh = NULL;
    mShadingOption = ShadingOption::SMOOTH_SHADING;
    mShadingEnabled = true;
    mShowWireframe = false;
    mSubdivScheme = SubdivideScheme::LOOP;
    mSubdivLevels = 1;

    Widget *tools = new Widget(window);
    tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                    Alignment::Middle, 0, 5));

    Button *b0 = new Button(tools, "Random Background Color");
    b0->setCallback([this]() { mCanvas->setBackgroundColor(Vector4i(rand() % 256, rand() % 256, rand() % 256, 255)); });

    nanogui::GLShader mShader;

    Window *anotherWindow = new Window(this, "Basic Functions");
    anotherWindow->setPosition(Vector2i(480, 15));
    anotherWindow->setLayout(new GroupLayout());

    //Message dialog demonstration
    /*
    new Label(anotherWindow, "Message dialog", "sans-bold");
    tools = new Widget(anotherWindow);
    tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                    Alignment::Middle, 0, 6));
    Button *b = new Button(tools, "Info");
    b->setCallback([&] {
        auto dlg = new MessageDialog(this, MessageDialog::Type::Information, "Title", "This is an information message");
        dlg->setCallback([](int result) { cout << "Dialog result: " << result << endl; });
    });
    b = new Button(tools, "Warn");
    b->setCallback([&] {
        auto dlg = new MessageDialog(this, MessageDialog::Type::Warning, "Title", "This is a warning message");
        dlg->setCallback([](int result) { cout << "Dialog result: " << result << endl; });
    });
    b = new Button(tools, "Ask");
    b->setCallback([&] {
        auto dlg = new MessageDialog(this, MessageDialog::Type::Warning, "Title", "This is a question message", "Yes", "No", true);
        dlg->setCallback([](int result) { cout << "Dialog result: " << result << endl; });
    });
    */
    new Label(anotherWindow, "File dialog", "sans-bold");
    tools = new Widget(anotherWindow);
    tools->setLayout(new BoxLayout(Orientation::Horizontal,
                                    Alignment::Middle, 0, 6));
    Button *b = new Button(tools, "Open");
    b->setCallback([this] {
        string filename = file_dialog({{"obj", "Mesh files"}}, false);
        if(filename.empty())
        {
            return;
        }
        
        if(mMesh)
        {
            delete mMesh;
        }
        mCanvas->setRotation(nanogui::Matrix4f::Identity());
        mCanvas->setTranslation({0, 0, 0});
        mCanvas->setScale({1, 1, 1});
        mMesh = new WingedEdgeMesh();
        mMesh->loadObj(filename);
        updateMeshData();
    });
    b = new Button(tools, "Save");
    b->setCallback([this] {
        string filename = file_dialog({{"obj", "Mesh files"}}, true);
        if(filename.empty())
        {
            return;
        }
        if(!mMesh)
        {
            cout << "Error: Please load a mesh first." << endl;
            return;
        }
        mMesh->saveObj(filename);
    });

    //This is how to implement a combo box, which is important in A1
    new Label(anotherWindow, "Shader Options", "sans-bold");
    ComboBox *combo = new ComboBox(anotherWindow, { "Flat shaded", "Smooth shaded"} );
    combo->setSelectedIndex(1);
    combo->setCallback([this](int value) {
        mShadingOption = value == 0 ? ShadingOption::FLAT_SHADING : ShadingOption::SMOOTH_SHADING;
        updateMeshData();
    });	

    new Label(anotherWindow, "Check box", "sans-bold");
    CheckBox *cb = new CheckBox(anotherWindow, "Shading",
        [this](bool state) { 
            mShadingEnabled = state;
            updateMeshData();
        }
    );
    cb->setChecked(true);
    cb = new CheckBox(anotherWindow, "Wireframe",
        [this](bool state) { 
            mShowWireframe = state;
            updateMeshData();
        }
    );

    // Subdivision Window
    Window *subdWindow = new Window(this, "Subdivision");
    subdWindow->setPosition(Vector2i(473, 280));
    subdWindow->setLayout(new GroupLayout());

    new Label(subdWindow, "Scheme", "sans-bold");
    combo = new ComboBox(subdWindow, { "Loop", "Butterfly"} );
    combo->setSelectedIndex(0);
    combo->setCallback([this](int value) {
        mSubdivScheme = value == 0 ? SubdivideScheme::LOOP : SubdivideScheme::BUTTERFLY;
    });

    new Label(subdWindow, "Number of Subdivision Levels", "sans-bold");

    Widget *panel = new Widget(subdWindow);
    panel->setLayout(new BoxLayout(Orientation::Horizontal,
                                    Alignment::Middle, 0, 20));    

    Slider *slider = new Slider(panel);
    slider->setValue(1);
    slider->setFixedWidth(80);
    slider->setRange(make_pair(1, 5));
    TextBox *textBox = new TextBox(panel);
    textBox->setFixedSize(Vector2i(60, 25));
    textBox->setValue("1");
    slider->setCallback([this, textBox](float value) {
        int subDivLevels = (int) value;
        textBox->setValue(std::to_string(subDivLevels));
        mSubdivLevels = subDivLevels;
    });
    textBox->setFontSize(20);
    textBox->setAlignment(TextBox::Alignment::Right);

    Button *subdButton = new Button(subdWindow, "Subdivide");
    subdButton->setCallback([&, subdButton] {
        if(!mMesh)
        {
            return;
        }
        subdButton->setEnabled(false);
        mMesh->subDivide(mSubdivScheme, mSubdivLevels);
        updateMeshData();
        subdButton->setEnabled(true);
	});

    // Decimation Window
    Window *deciWindow = new Window(this, "Decimation");
    deciWindow->setPosition(Vector2i(475, 490));
    deciWindow->setLayout(new GroupLayout());

    panel = new Widget(deciWindow);
    panel->setLayout(new BoxLayout(Orientation::Horizontal,
                                    Alignment::Middle, 0, 20));    

    new Label(panel, "k" + string(30, ' '), "sans-bold");

    IntBox<int> *kTextBox = new IntBox<int>(panel);
    kTextBox->setFixedSize(Vector2i(60, 25));
    kTextBox->setValue(8);
    kTextBox->setFontSize(20);
    kTextBox->setAlignment(TextBox::Alignment::Right);
    kTextBox->setEditable(true);

    panel = new Widget(deciWindow);
    panel->setLayout(new BoxLayout(Orientation::Horizontal,
                                    Alignment::Middle, 0, 20));    

    new Label(panel, "Edges to collapse", "sans-bold");

    IntBox<int> *nTextBox = new IntBox<int>(panel);
    nTextBox->setFixedSize(Vector2i(60, 25));
    nTextBox->setValue(1);
    nTextBox->setFontSize(20);
    nTextBox->setAlignment(TextBox::Alignment::Right);
    nTextBox->setEditable(true);

    Button *deciButton = new Button(deciWindow, "Decimate");
    deciButton->setCallback([&, deciButton, kTextBox, nTextBox] {
        if(!mMesh)
        {
            return;
        }
        deciButton->setEnabled(false);
        int k = kTextBox->value();
        int n = nTextBox->value();
        mMesh->decimate(k, n);
        updateMeshData();
        deciButton->setEnabled(true);
    }); 

    Button *quitButton = new Button(deciWindow, "Quit");
    quitButton->setCallback([&] {
        if(mMesh)
        {
            delete mMesh;
        }
        mMesh = NULL;
        nanogui::leave();
    }); 

	//Method to assemble the interface defined before it is called
        performLayout();
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }


private:
    MyGLCanvas *mCanvas;
    WingedEdgeMesh *mMesh;
    ShadingOption mShadingOption;
    bool mShadingEnabled;
    bool mShowWireframe;
    SubdivideScheme mSubdivScheme;
    int mSubdivLevels;

    void updateMeshData()
    {
        if(!mMesh)
        {
            return;
        }

        MatrixXf newPositions, newNormals, newColors;
        mMesh->getMeshData(mShadingOption, newPositions, newNormals, newColors);

		mCanvas->updateMeshPositions(newPositions);
        mCanvas->updateMeshNormals(newNormals);
        mCanvas->updateMeshColors(newColors);
        mCanvas->setDrawFaces(mShadingEnabled);
        mCanvas->setDrawLines(mShowWireframe);
    }
};

int main(int /* argc */, char ** /* argv */) {
    try {
        nanogui::init();

            /* scoped variables */ {
            nanogui::ref<ExampleApplication> app = new ExampleApplication();
            app->drawAll();
            app->setVisible(true);
            nanogui::mainloop();
        }

        nanogui::shutdown();
    } catch (const std::runtime_error &e) {
        std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
        #if defined(_WIN32)
            MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
        #else
            std::cerr << error_msg << endl;
        #endif
        return -1;
    }

    return 0;
}
