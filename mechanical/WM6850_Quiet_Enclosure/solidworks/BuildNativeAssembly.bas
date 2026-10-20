Attribute VB_Name = "BuildNativeAssembly"
Option Explicit
' Optional conversion helper. NOT executed or verified in SolidWorks.
' In SolidWorks: Tools > Macro > New, then import this .bas in the VBA editor.
' Requires the SolidWorks and SolidWorks Constant type-library references.
' Creates a new timestamped folder; does not overwrite existing native files.

Sub main()
    On Error GoTo Failed
    Dim swApp As SldWorks.SldWorks
    Dim model As SldWorks.ModelDoc2, assyModel As SldWorks.ModelDoc2
    Dim assy As SldWorks.AssemblyDoc, comp As SldWorks.Component2
    Dim importData As Object, fs As Object, item As Object
    Dim inputFiles As Collection, nativePaths As Collection
    Dim src As String, output As String, template As String, path As Variant
    Dim errors As Long, warnings As Long, ok As Boolean
    Dim mathUtil As SldWorks.MathUtility, transform As SldWorks.MathTransform
    Dim values(15) As Double, variantValues As Variant
    Set swApp = Application.SldWorks
    Set fs = CreateObject("Scripting.FileSystemObject")
    src = InputBox("Full path to the WM6850_Quiet_Enclosure project folder:", _
                   "Import enclosure STEP parts", _
                   "D:\Github\CPAP_InitialCode\mechanical\WM6850_Quiet_Enclosure")
    If Len(src) = 0 Then Exit Sub
    If Not fs.FolderExists(src & "\cad\step_parts") Then Err.Raise 513, , "STEP parts folder not found."
    template = swApp.GetUserPreferenceStringValue(swUserPreferenceStringValue_e.swDefaultTemplateAssembly)
    If Not fs.FileExists(template) Then Err.Raise 514, , "Set a valid default assembly template in SolidWorks first."
    output = src & "\solidworks\native_" & Format(Now, "yyyymmdd_hhnnss")
    If Not fs.FolderExists(src & "\solidworks") Then fs.CreateFolder src & "\solidworks"
    If fs.FolderExists(output) Then Err.Raise 515, , "Output folder already exists; retry after one second."
    fs.CreateFolder output
    Set inputFiles = New Collection
    Set nativePaths = New Collection
    For Each item In fs.GetFolder(src & "\cad\step_parts").Files
        If LCase(fs.GetExtensionName(item.Name)) = "step" Then inputFiles.Add item.Path
    Next
    If inputFiles.Count <> 22 Then Err.Raise 516, , "Expected 22 component STEP files for revision A."
    For Each path In inputFiles
        Set importData = swApp.GetImportFileData(CStr(path))
        errors = 0
        Set model = swApp.LoadFile4(CStr(path), "r", importData, errors)
        If model Is Nothing Then Err.Raise 517, , "Cannot import " & CStr(path) & "; code " & errors
        If model.GetType <> swDocumentTypes_e.swDocPART Then Err.Raise 518, , "Expected an imported part: " & CStr(path)
        Dim nativeFile As String
        nativeFile = output & "\" & fs.GetBaseName(CStr(path)) & ".SLDPRT"
        errors = 0: warnings = 0
        ok = model.Extension.SaveAs(nativeFile, swSaveAsVersion_e.swSaveAsCurrentVersion, _
              swSaveAsOptions_e.swSaveAsOptions_Silent, Nothing, errors, warnings)
        If Not ok Or errors <> 0 Then Err.Raise 519, , "Part save failed: " & nativeFile & "; code " & errors
        nativePaths.Add nativeFile
        ' Leave imported documents loaded: AddComponent5 requires loaded parts.
    Next
    Set assyModel = swApp.NewDocument(template, 0, 0, 0)
    If assyModel Is Nothing Then Err.Raise 520, , "Could not create assembly from template."
    Set assy = assyModel
    Set mathUtil = swApp.GetMathUtility
    values(0) = 1: values(4) = 1: values(8) = 1: values(12) = 1
    variantValues = values
    Set transform = mathUtil.CreateTransform(variantValues)
    For Each path In nativePaths
        Set comp = assy.AddComponent5(CStr(path), _
           swAddComponentConfigOptions_e.swAddComponentConfigOptions_CurrentSelectedConfig, _
           "", False, "", 0, 0, 0)
        If comp Is Nothing Then Err.Raise 521, , "Could not add component: " & CStr(path)
        assyModel.ClearSelection2 True
        ok = comp.Select4(False, Nothing, False)
        If Not ok Then Err.Raise 522, , "Could not select imported component."
        assy.UnfixComponent
        Set comp.Transform2 = transform
        assy.FixComponent
        assyModel.ClearSelection2 True
    Next
    assyModel.ForceRebuild3 False
    assyModel.ViewZoomtofit2
    errors = 0: warnings = 0
    ok = assyModel.Extension.SaveAs(output & "\WM6850_Quiet_Enclosure.SLDASM", _
         swSaveAsVersion_e.swSaveAsCurrentVersion, swSaveAsOptions_e.swSaveAsOptions_Silent, _
         Nothing, errors, warnings)
    If Not ok Or errors <> 0 Then Err.Raise 523, , "Assembly save failed; code " & errors
    MsgBox "Native files saved in:" & vbCrLf & output & vbCrLf & _
           "Verify 22 components, base 220 x 160 x 104 mm and blower fit. " & _
           "Imported solids have no native feature history.", vbInformation
    Exit Sub
Failed:
    MsgBox "Conversion stopped: " & Err.Description & vbCrLf & _
           "Any partial files remain in the new folder. Use the manual import instructions in README.md.", vbExclamation
End Sub
