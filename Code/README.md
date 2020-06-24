# This directory contains all Grasshopper files

# "Main Interface.gh" is to create main window interface. Different child window interface should first be developed in different folders. When the child window interface is ready, you can add the child window grasshopper code to "Main Interface.gh".

# Within each folder, it should contain the following files:
(1)Grasshopper files: Please name the test Grasshopper files as the following format: <Motion name>_<developer's initials>_test_<version #>.gh # Example: Intermediate_ContinuousMoving_Rotate_YL_test_V1.gh
(2)Library: It contains all the components that you developed yourself in visual studio. If it's your first time to create your own component in visual studio, you can visit #https://developer.rhino3d.com/guides/grasshopper/installing-tools-windows/
#https://developer.rhino3d.com/guides/grasshopper/your-first-component-windows/


这个文件夹里是新框架目前的所有代码，分别是类库，motion solver和utilities。
其中motion solver包含了两个已经初步写完的motion，分别是InstantExtension和ContinuousLinearMovingByPress，他们各自都引用了Kinergy类库。
utilities是之后要完善的部分，包含了所有辅助工具电池，目前只写了一个EntityReader,之后simulation的电池也会写在这里面。

如果需要加载电池，只需要将每个项目的bin文件夹里的gha文件拷贝到 Program Files\Rhino 6\Plug-ins\Grasshopper\Components文件夹里，
另外再拷贝Kinergy.dll(只要引用的都是用同一个版本的类库，各个项目文件夹里的dll就都是一样的，有一份就行)，以及KinergyResources文件夹到上面的路径
之后启动grasshopper，就能看到电池了

如果电池报错说dll找不到或者版本不对，是因为不同电池对kinergy的引用版本不统一，这种情况下打开各个电池的vs项目，
确保kinergy的引用有效，然后重新生成一次就行了
现有版本小bug应该不少，我这几天会密集更新


#5月28日更新：为了实现小窗里的simulation，加入了HumanUI for Kinergy项目。这个项目是在HumanUI的基础上更改的，只加入了一个simulation电池，其余功能不变，需要使用的话，建议移除原有的HumanUI文件，然后将HumanUI for Kinergy下的bin路径加入到GrasshopperDeveloperSetting里面。

#6月17日更新：按照Liang的建议对数个命名进行修改

#6月24日更新：针对弯曲模型的InstantExtension已经完成。另外，由于原先多个独立的项目之间存在类的声明不互通的情况，因此将原先的HumanUI for Kinergy，MotionSolver以及KinergyUtilities三个项目进行了合并，归到KinergyApp文件夹下。原有的三个项目文件夹丢到old code里面进行留档。目前代码分为KinergyApp和KinergyClassLibrary两个项目，减少了每次修改之后生成操作的次数。
