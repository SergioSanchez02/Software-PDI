from __future__ import print_function
import sys
import sys, re, os
import numpy as np

from __main__ import vtk, qt, ctk, slicer
try:
  NUMPY_AVAILABLE = True
  import vtk.util.numpy_support
except:
  NUMPY_AVAILABLE = False
from MultiVolumeImporterLib.Helper import Helper

class Registro:
  def __init__(self, parent):
    ### Informacion acerca del codigo, que se permite leer en Slicer 3d
    parent.title = "Registration"
    parent.categories = ["Examples"]
    parent.dependencies = []
    parent.contributors = ["Marco Mayo",
                           "Sergio Sanchez",
                           "Juan Betancur"] 
    parent.helpText = """
    Regitration multivolumen.
    """
    parent.acknowledgementText = """
    Digital image processing.""" 
    self.parent = parent


class RegistroWidget:
  def __init__(self, parent = None):
    if not parent:
      self.parent = slicer.qMRMLWidget()
      self.parent.setLayout(qt.QVBoxLayout())
      self.parent.setMRMLScene(slicer.mrmlScene)
    else:
      self.parent = parent
    self.layout = self.parent.layout()
    if not parent:
      self.setup()
      self.parent.show()

  def setup(self):
    
    # Collapsible button
    self.dummyCollapsibleButton = ctk.ctkCollapsibleButton()
    self.dummyCollapsibleButton.text = "Import MultiVolume"
    self.layout.addWidget(self.dummyCollapsibleButton)
    self.dummyFormLayout = qt.QFormLayout(self.dummyCollapsibleButton)

    # add input directory selector
    label = qt.QLabel('Input directory:')
    self.__fDialog = ctk.ctkDirectoryButton()
    self.__fDialog.caption = 'Input directory'
    self.dummyFormLayout.addRow(label, self.__fDialog)

    label = qt.QLabel('Output node:')
    self.__mvSelector = slicer.qMRMLNodeComboBox()
    self.__mvSelector.nodeTypes = ['vtkMRMLMultiVolumeNode']
    self.__mvSelector.setMRMLScene(slicer.mrmlScene)
    self.__mvSelector.connect('mrmlSceneChanged(vtkMRMLScene*)', self.onMRMLSceneChanged)
    self.__mvSelector.addEnabled = 1
    self.dummyFormLayout.addRow(label, self.__mvSelector)

    importButton = qt.QPushButton("Import")
    importButton.toolTip = "Import the contents of the directory as a MultiVolume"
    self.dummyFormLayout.addWidget(importButton)
    importButton.connect('clicked(bool)', self.onImportButtonClicked)  


#########################################################################################################
##########################################################################################################
    
    
    # Se crea el boton plegable, donde estaran las funciones del registro
    self.RegistrationCollapsibleButton = ctk.ctkCollapsibleButton()
    self.RegistrationCollapsibleButton.text = "Registration" ## Label del boton
    self.layout.addWidget(self.RegistrationCollapsibleButton) ## Se agrega el boton 

    # Diseno de los botones y funciones dentro del boton plegable
    self.RegistrationFormLayout = qt.QFormLayout(self.RegistrationCollapsibleButton)

    # Seleccion del multi Volumen
    self.inputFrame = qt.QFrame(self.RegistrationCollapsibleButton) #Se crea el frame
    self.inputFrame.setLayout(qt.QHBoxLayout())
    self.RegistrationFormLayout.addWidget(self.inputFrame) #Se agrega el frame 
    self.inputSelector1 = qt.QLabel("Input Volume: ", self.inputFrame) 
    self.inputFrame.layout().addWidget(self.inputSelector1)
    self.inputSelector1 = slicer.qMRMLNodeComboBox(self.inputFrame) #Se crea el boton
    self.inputSelector1.nodeTypes = ( ("vtkMRMLMultiVolumeNode"), "" ) #Se indica el tipo de volumen que se requiere
    self.inputSelector1.addEnabled = False
    self.inputSelector1.removeEnabled = False
    self.inputSelector1.setMRMLScene( slicer.mrmlScene )
    self.inputFrame.layout().addWidget(self.inputSelector1) #Se agrega el boton


    
    # Seleccion del Volumen fijo
    self.inputFrame = qt.QFrame(self.RegistrationCollapsibleButton) #Se crea el frame
    self.inputFrame.setLayout(qt.QHBoxLayout())
    self.RegistrationFormLayout.addWidget(self.inputFrame) #Se agrega el frame 
    self.inputSelector2 = qt.QLabel("Input Volume fixed: ", self.inputFrame) 
    self.inputFrame.layout().addWidget(self.inputSelector2)
    self.inputSelector2 = slicer.qMRMLNodeComboBox(self.inputFrame) #Se crea el boton
    self.inputSelector2.nodeTypes = ( ("vtkMRMLScalarVolumeNode"), "" ) #Se indica el tipo de volumen que se requiere
    self.inputSelector2.addEnabled = False
    self.inputSelector2.removeEnabled = False
    self.inputSelector2.setMRMLScene( slicer.mrmlScene )
    self.inputFrame.layout().addWidget(self.inputSelector2) #Se agrega el boton

    self.sharpen = qt.QCheckBox("Filter", self.RegistrationCollapsibleButton)
    self.sharpen.toolTip = "When checked, laplacian filter input volume"
    self.sharpen.checked = True
    self.RegistrationFormLayout.addWidget(self.sharpen)


    ##########
    ## Boton desplegable para escoger el tipo de transformacion

    self.inputFrame = qt.QFrame(self.RegistrationCollapsibleButton)
    self.inputFrame.setLayout(qt.QHBoxLayout())
    self.RegistrationFormLayout.addWidget(self.inputFrame)
    self.TransMenu = qt.QLabel("Input Type Transformation: ", self.inputFrame)
    self.inputFrame.layout().addWidget(self.TransMenu)    
    self.TransMenu = ctk.ctkComboBox()
    ##Se agrega las posibles opciones de transformada
    self.TransMenu.addItem('Rigid','1')
    self.TransMenu.addItem('Affine','2')
    self.TransMenu.addItem('BSpline','3')
    self.RegistrationFormLayout.addWidget(self.TransMenu)    

    global Trans
    global vector

    # Boton para realizar el Registro
    ApplyButton = qt.QPushButton("Apply")
    ApplyButton.toolTip = "Run the Registration."
    self.RegistrationFormLayout.addWidget(ApplyButton)
    ApplyButton.connect('clicked(bool)', self.onApply)

    # Set local var as instance attribute
    self.ApplyButton = ApplyButton    

#########################################################################################################
##########################################################################################################
    # Se crea el boton plegable
    self.ROICollapsibleButton = ctk.ctkCollapsibleButton()
    self.ROICollapsibleButton.text = "Aply ROI and graph" ## Label del boton
    self.layout.addWidget(self.ROICollapsibleButton) ## Se agrega el boton 

    # Diseno de los botones y funciones dentro del boton plegable
    self.ROIFormLayout = qt.QFormLayout(self.ROICollapsibleButton)

    #Seleccion de la ROI
    self.inputFrame = qt.QFrame(self.ROICollapsibleButton) #Se crea el frame
    self.inputFrame.setLayout(qt.QHBoxLayout())
    self.ROIFormLayout.addWidget(self.inputFrame) #Se agrega el frame 
    self.inputSelector2 = qt.QLabel("Input label ROI: ", self.inputFrame) 
    self.inputFrame.layout().addWidget(self.inputSelector2)
    self.inputSelector2 = slicer.qMRMLNodeComboBox(self.inputFrame) #Se crea el boton
    self.inputSelector2.nodeTypes = ( ("vtkMRMLScalarVolumeNode"), "" ) #Se indica el tipo de volumen que se requiere
    self.inputSelector2.addEnabled = False
    self.inputSelector2.removeEnabled = False
    self.inputSelector2.setMRMLScene( slicer.mrmlScene )
    self.inputFrame.layout().addWidget(self.inputSelector2) #Se agrega el boton

    self.graph = qt.QCheckBox("Graph", self.ROICollapsibleButton)
    self.graph.toolTip = "When checked, laplacian filter input volume"
    self.graph.checked = True
    self.ROIFormLayout.addWidget(self.graph)

    # Boton aplicar
    ApplyG = qt.QPushButton("Apply")
    ApplyG.toolTip = "Run"
    self.ROIFormLayout.addWidget(ApplyG)
    ApplyG.connect('clicked(bool)', self.ApplyClicked)

    # Set local var as instance attribute
    #self.ApplyG = ApplyG    




#########################################################################################################
##########################################################################################################

  def enter(self):
    return

  def onMRMLSceneChanged(self, mrmlScene):
    self.__mvSelector.setMRMLScene(slicer.mrmlScene)
    return

  def humanSort(self,l):
    """ Sort the given list in the way that humans expect. 
        Conributed by Yanling Liu
    """ 
    convert = lambda text: int(text) if text.isdigit() else text 
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
    l.sort( key=alphanum_key )

  def onImportButtonClicked(self):
    # check if the output container exists
    mvNode = self.__mvSelector.currentNode()
    if mvNode == None:
      self.__status.text = 'Status: Select output node!'
      return

    # Series of frames alpha-ordered, all in the input directory
    # Assume here that the last mode in the list is for parsing a list of
    # non-DICOM frames

    fileNames = []    # file names on disk
    frameList = []    # frames as MRMLScalarVolumeNode's
    frameFolder = ""
    volumeLabels = vtk.vtkDoubleArray()
    frameLabelsAttr = ''
    frameFileListAttr = ''
    dicomTagNameAttr = 'NA'
    dicomTagUnitsAttr = 'na'
    teAttr = 1
    trAttr = 1
    faAttr = 1

    # each frame is saved as a separate volume
    # first filter valid file names and sort alphabetically
    frames = []
    frame0 = None
    inputDir = self.__fDialog.directory
    for f in os.listdir(inputDir):
      if not f.startswith('.'):
        fileName = inputDir+'/'+f
        fileNames.append(fileName)
    self.humanSort(fileNames)

    for fileName in fileNames:
      (s,f) = self.readFrame(fileName)
      if s:
        if not frame0:
          frame0 = f
          frame0Image = frame0.GetImageData()
          frame0Extent = frame0Image.GetExtent()
        else:
          frameImage = f.GetImageData()
          frameExtent = frameImage.GetExtent()
          if frameExtent[1]!=frame0Extent[1] or frameExtent[3]!=frame0Extent[3] or frameExtent[5]!=frame0Extent[5]:
            continue
        frames.append(f)

    nFrames = len(frames)
    print('Successfully read '+str(nFrames)+' frames')

    if nFrames == 1:
      print('Single frame dataset - not reading as multivolume!')
      return

    volumeLabels.SetNumberOfTuples(nFrames)
    volumeLabels.SetNumberOfComponents(1)
    volumeLabels.Allocate(nFrames)
    for i in range(nFrames):
      frameId = 0+1*i
      volumeLabels.SetComponent(i, 0, frameId)
      frameLabelsAttr += str(frameId)+','
    frameLabelsAttr = frameLabelsAttr[:-1]

    # allocate multivolume
    mvImage = vtk.vtkImageData()
    mvImage.SetExtent(frame0Extent)
    if vtk.VTK_MAJOR_VERSION <= 5:
      mvImage.SetNumberOfScalarComponents(nFrames)
      mvImage.SetScalarType(frame0.GetImageData().GetScalarType())
      mvImage.AllocateScalars()
    else:
      mvImage.AllocateScalars(frame0.GetImageData().GetScalarType(), nFrames)

    extent = frame0.GetImageData().GetExtent()
    numPixels = float(extent[1]+1)*(extent[3]+1)*(extent[5]+1)*nFrames
    scalarType = frame0.GetImageData().GetScalarType()
    print('Will now try to allocate memory for '+str(numPixels)+' pixels of VTK scalar type'+str(scalarType))
    print('Memory allocated successfully')
    mvImageArray = vtk.util.numpy_support.vtk_to_numpy(mvImage.GetPointData().GetScalars())

    mat = vtk.vtkMatrix4x4()
    frame0.GetRASToIJKMatrix(mat)
    mvNode.SetRASToIJKMatrix(mat)
    frame0.GetIJKToRASMatrix(mat)
    mvNode.SetIJKToRASMatrix(mat)

    for frameId in range(nFrames):
      # TODO: check consistent size and orientation!
      frame = frames[frameId]
      frameImage = frame.GetImageData()
      frameImageArray = vtk.util.numpy_support.vtk_to_numpy(frameImage.GetPointData().GetScalars())
      mvImageArray.T[frameId] = frameImageArray

    mvDisplayNode = slicer.mrmlScene.CreateNodeByClass('vtkMRMLMultiVolumeDisplayNode')
    mvDisplayNode.SetScene(slicer.mrmlScene)
    slicer.mrmlScene.AddNode(mvDisplayNode)
    mvDisplayNode.SetReferenceCount(mvDisplayNode.GetReferenceCount()-1)
    mvDisplayNode.SetDefaultColorMap()

    mvNode.SetAndObserveDisplayNodeID(mvDisplayNode.GetID())
    mvNode.SetAndObserveImageData(mvImage)
    mvNode.SetNumberOfFrames(nFrames)

    mvNode.SetLabelArray(volumeLabels)
    mvNode.SetLabelName('na')

    mvNode.SetAttribute('MultiVolume.FrameLabels',frameLabelsAttr)
    mvNode.SetAttribute('MultiVolume.NumberOfFrames',str(nFrames))
    mvNode.SetAttribute('MultiVolume.FrameIdentifyingDICOMTagName',dicomTagNameAttr)
    mvNode.SetAttribute('MultiVolume.FrameIdentifyingDICOMTagUnits',dicomTagUnitsAttr)

    if dicomTagNameAttr == 'TriggerTime' or dicomTagNameAttr == 'AcquisitionTime':
      if teTag != '':
        mvNode.SetAttribute('MultiVolume.DICOM.EchoTime',teTag)
      if trTag != '':
        mvNode.SetAttribute('MultiVolume.DICOM.RepetitionTime',trTag)
      if faTag != '':
        mvNode.SetAttribute('MultiVolume.DICOM.FlipAngle',faTag)

    mvNode.SetName(str(nFrames)+' frames MultiVolume')
    Helper.SetBgFgVolumes(mvNode.GetID(),None)

  def readFrame(self,file):
    sNode = slicer.vtkMRMLVolumeArchetypeStorageNode()
    sNode.ResetFileNameList()
    sNode.SetFileName(file)
    sNode.SetSingleFile(1)
    frame = slicer.vtkMRMLScalarVolumeNode()
    success = sNode.ReadData(frame)
    return (success,frame)

  # leave no trace of the temporary nodes
  def annihilateScalarNode(self, node):
    dn = node.GetDisplayNode()
    sn = node.GetStorageNode()
    node.SetAndObserveDisplayNodeID(None)
    node.SetAndObserveStorageNodeID(None)
    slicer.mrmlScene.RemoveNode(dn)
    slicer.mrmlScene.RemoveNode(sn)
    slicer.mrmlScene.RemoveNode(node)


    
##############################################################################################
    ######################################################################################

  def ApplyClicked(self):
    global vector
    vector2=[]
    print(vector)
    ROIlabel=self.inputSelector2.currentNode()

    for i in range(0,len(vector)):
      #se crea el volumen de salida de la operacion y se anade a la escena
      vol = slicer.vtkMRMLScalarVolumeNode();
      vol.SetName('salida') 
      slicer.mrmlScene.AddNode(vol)

      slicer.mrmlScene.AddNode(vector[i])

      #parametros para la operacion de registro
      parameters = {}
      parameters['inputVolume1'] = ROIlabel.GetID() #dos volumenes de la escena, uno de ellos debe ser la mascara creada en el EDITOR
      parameters['inputVolume2'] = vector[i].GetID()
      parameters['outputVolume'] = vol;
      cliNode = slicer.cli.run( slicer.modules.multiplyscalarvolumes,None,parameters, wait_for_completion=True)

      a = slicer.util.arrayFromVolume(vol)
      #vector2.append(np.mean(a[:]))
      vector2.append(np.mean(a[a>0]))

############################# Graficas ################################################################    

    # Switch to a layout (24) that contains a Chart View to initiate the construction of the widget and Chart View Node
    lns = slicer.mrmlScene.GetNodesByClass('vtkMRMLLayoutNode')
    lns.InitTraversal()
    ln = lns.GetNextItemAsObject()
    ln.SetViewArrangement(24)

    # Get the Chart View Node
    cvns = slicer.mrmlScene.GetNodesByClass('vtkMRMLChartViewNode')
    cvns.InitTraversal()
    cvn = cvns.GetNextItemAsObject()

    # Create an Array Node and add some data
    dn = slicer.mrmlScene.AddNode(slicer.vtkMRMLDoubleArrayNode())
    a = dn.GetArray()
    a.SetNumberOfTuples(27)
    x = range(0,26)
    for i in range(len(x)):
        a.SetComponent(i, 0, i*11)
        a.SetComponent(i, 1, vector2[i])
        a.SetComponent(i, 2, 0)

    # Create a Chart Node.
    cn = slicer.mrmlScene.AddNode(slicer.vtkMRMLChartNode())

    # Add the Array Nodes to the Chart. The first argument is a string used for the legend and to refer to the Array when setting properties.
    cn.AddArray('A double array', dn.GetID())
    #cn.AddArray('Another double array', dn2.GetID())

    # Set a few properties on the Chart. The first argument is a string identifying which Array to assign the property. 
    # 'default' is used to assign a property to the Chart itself (as opposed to an Array Node).
    cn.SetProperty('default', 'title', 'Intensidad Vs Tiempo')
    cn.SetProperty('default', 'xAxisLabel', 'Tiempo')
    cn.SetProperty('default', 'yAxisLabel', 'Intensidad')

    # Tell the Chart View which Chart to display
    cvn.SetChartNodeID(cn.GetID())

##############################################################################################
    ######################################################################################

  def onApply(self):
    global Trans
    global vector

    Trans=self.TransMenu.currentText ## Extracion de informacion del boton desplegable 
    Trans=str(Trans) ## Conversion a tipo string

    
    volumen4D = self.inputSelector1.currentNode() ##Extraer informacion de la seleccion del multi volumen
    if not (volumen4D):
      qt.QMessageBox.critical(slicer.util.mainWindow(),'Registration', 'Input volumes is required for Registration')#Mensaje de advertencia de que no se escogio ningun multi volumen
      return
    
    escena = slicer.mrmlScene ##Se importa la escena
    imagenvtk4D = volumen4D.GetImageData()
    numero_imagenes = volumen4D.GetNumberOfFrames() ##Se obtiene el numero de volumenes que componen la imagen 4D
            
    #filtro vtk para descomponer un volumen 4D
    extract1 = vtk.vtkImageExtractComponents()
    extract1.SetInputData(imagenvtk4D)

    #matriz de transformacion
    ras2ijk = vtk.vtkMatrix4x4()
    ijk2ras = vtk.vtkMatrix4x4()

    #le solicitamos al volumen original que nos devuelva sus matrices
    volumen4D.GetRASToIJKMatrix(ras2ijk)
    volumen4D.GetIJKToRASMatrix(ijk2ras)

    #creo un volumen nuevo
    volumenFijo = slicer.vtkMRMLScalarVolumeNode()
    #le asigno las transformaciones
    volumenFijo.SetRASToIJKMatrix(ras2ijk)
    volumenFijo.SetIJKToRASMatrix(ijk2ras)
    #le asigno el volumen 3D fijo
    volumenFijo=self.inputSelector2.currentNode()
    volumenFijo.SetName('fijo')

    #anado el nuevo volumen a la escena
    escena.AddNode(volumenFijo)

    #Se crea nuevo volumnen y se agrega a la escena, este sera el volume de salida ya con el registro
    volumenSalida = slicer.vtkMRMLScalarVolumeNode()
    escena.AddNode(volumenSalida)
             

    for i in range(0,numero_imagenes):

      #creamos la transformada para alinear los volumenes
      #Se pone el condicional para los 3 tipos de transformadas 
      if (Trans=='Rigid'):
        transformadaSalida = slicer.vtkMRMLLinearTransformNode()
        transformadaSalida.SetName('Transformada')## Se nombran las transformadas 
        slicer.mrmlScene.AddNode(transformadaSalida)
      elif (Trans=='BSpline'):
        transformadaSalida = slicer.vtkMRMLBSplineTransformNode()
        transformadaSalida.SetName('Transformada')
        slicer.mrmlScene.AddNode(transformadaSalida)
      elif (Trans=='Affine'):
        transformadaSalida = slicer.vtkMRMLBSplineTransformNode()
        transformadaSalida.SetName('Transformada')
        slicer.mrmlScene.AddNode(transformadaSalida)

      
      # extraigo la imagen movil
      imagen_movil = extract1.SetComponents(i) 
      extract1.Update()
      
      #Creo un volumen movil, y realizamos el mismo procedimiento que con el fijo
      volumenMovil = slicer.vtkMRMLScalarVolumeNode();
      volumenMovil.SetRASToIJKMatrix(ras2ijk)
      volumenMovil.SetIJKToRASMatrix(ijk2ras)
      volumenMovil.SetAndObserveImageData(extract1.GetOutput())
      volumenMovil.SetName('movil')
      escena.AddNode(volumenMovil)

      #parametros para la operacion de registro
      parameters = {}
      parameters['fixedVolume'] = volumenFijo.GetID()
      parameters['movingVolume'] = volumenMovil.GetID()
      parameters['transformType'] = Trans
      parameters['outputTransform'] = transformadaSalida.GetID()
      parameters['outputVolume'] = volumenSalida.GetID()

      cliNode = slicer.cli.run( slicer.modules.brainsfit,None,parameters, wait_for_completion=True)#Funcion para realizar el registro


      if self.sharpen.checked:
        laplacian = vtk.vtkImageLaplacian()
        laplacian.SetInputData(volumenSalida.GetImageData())
        laplacian.SetDimensionality(3)
        laplacian.Update()
        ijkToRAS = vtk.vtkMatrix4x4()
        volumenSalida.GetIJKToRASMatrix(ijkToRAS)
        outputVolume = slicer.vtkMRMLScalarVolumeNode()
        outputVolume.SetIJKToRASMatrix(ijkToRAS)
        outputVolume.SetAndObserveImageData(laplacian.GetOutput())

      if i==0:
        vector=[outputVolume]
      else:
          vector.append(outputVolume)

      #print(len(vector))

      slicer.util.saveNode(outputVolume,'E:\Documentos_2\Onceavo Semestre\Imagenes\Tarea1\Nueva carpeta\Volumen'+str(i)+'.nhdr')#Se guarda el volumen de salida
      ##Con los volumenes guardados, posteriormente se puede armar un volumen 4D
      
###############################################################################
########################################################################3#######




