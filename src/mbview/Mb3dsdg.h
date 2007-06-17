
/* Begin user code block <abstract> */
/* End user code block <abstract> */

/**
 * README: Portions of this file are merged at file generation
 * time. Edits can be made *only* in between specified code blocks, look
 * for keywords <Begin user code> and <End user code>.
 */
/*
 * Generated by the ICS Builder Xcessory (BX).
 *
 * BuilderXcessory Version 6.1.3
 * Code Generator Xcessory 6.1.3 (08/19/04) CGX Scripts 6.1 Motif 2.1 
 *
 */
#ifndef Mb3dsdg_H
#define Mb3dsdg_H

/**
 * Forward declare the class data pointer type so that it can
 * easily be used as a parameter to class functions and data members.
 */
typedef struct _Mb3dsdgData *Mb3dsdgDataPtr;

/**
 * Globally included information.
 */


/**
 * Class specific includes.
 */


typedef struct _Mb3dsdgData
{
    /*
     * Classes created by this class.
     */
    
    /*
     * Widgets created by this class.
     */
    Widget Mb3dsdg;
    Widget text_status;
    Widget pushButton_dismiss;
    Widget drawingArea;
    Widget radioBox_soundingsmode;
    Widget toggleButton_mouse_toggle;
    Widget toggleButton_mouse_pick;
    Widget toggleButton_mouse_erase;
    Widget toggleButton_mouse_restore;
    Widget toggleButton_mouse_grab;
    Widget toggleButton_mouse_info;
    
    /**
     * All methods and data..
     */
} Mb3dsdgData;

/*
 * Function: Mb3dsdgCreate()
 *		The creation for class Mb3dsdg.
 */
Mb3dsdgDataPtr Mb3dsdgCreate(Mb3dsdgDataPtr, Widget, String, ArgList, Cardinal);

/**
 * Set routines for exposed resources.
 */
#endif