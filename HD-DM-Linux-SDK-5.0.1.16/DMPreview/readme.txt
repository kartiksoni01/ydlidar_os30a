============================================================================================
I.EYSD Enviroment Setting (Ubuntu x86_64)
============================================================================================
Please run below command:

sudo sh setup_env.sh

============================================================================================
II.Build DMPreview
============================================================================================

(1) Using QtCreator

1-1. Open QtCreator.
1-2. Click "Open Project".
1-3-1. Choose "./DMPreview/DMPreview_x86_64.pro". (for X86_64)
1-3-2. Choose "./DMPreview/DMPreview_TX2.pro". (for NVIDIA TX2/NANO)
1-4. Click "Configure Project".
1.5. Build & Run project.

(2) Using QMake


2-1. rm -f .qmake.stash
2-2-1. qmake DMPreview_x86_64.pro (for X86_64)
2-2-2. qmake  DMPreview_TX2.pro (for NVIDIA TX2/NANO)
2-3. make
2-4-1. cp -f ./DMPreview ../bin/DMPreview_X86 (for X86_64)
2-4-2. cp -f ./DMPreview ../bin/DMPreview_TX2 (for NVIDIA TX2/NANO)
2-5. cd ../bin
2-6. ./run_DMPreview.sh (For X86_64, select '1'. For NVIDIA TX2 and NVIDIA NANO, select '4')

============================================================================================
