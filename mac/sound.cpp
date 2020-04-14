/******************************************************************************\
 * Copyright (c) 2004-2020
 *
 * Author(s):
 *  Volker Fischer
 *
 ******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
\******************************************************************************/
#include <AudioToolbox/AudioToolbox.h>
#include "sound.h"


/* Implementation *************************************************************/
CSound::CSound ( void       (*fpNewProcessCallback) ( CVector<short>& psData, void* arg ),
                 void*      arg,
                 const int  iCtrlMIDIChannel,
                 const bool bNoAutoJackConnect ) :
    CSoundBase ( "CoreAudio", true, fpNewProcessCallback, arg, iCtrlMIDIChannel, bNoAutoJackConnect ),
    midiInPortRef ( static_cast<MIDIPortRef> ( NULL ) )
{
    // Apple Mailing Lists: Subject: GUI Apps should set kAudioHardwarePropertyRunLoop
    // in the HAL, From: Jeff Moore, Date: Fri, 6 Dec 2002
    // Most GUI applciations have several threads on which they receive
    // notifications already, so the having the HAL's thread around is wasteful.
    // Here is what you should do: On the thread you want the HAL to use for
    // notifications (for most apps, this will be the main thread), add the
    // following lines of code:
    // tell the HAL to use the current thread as it's run loop
    CFRunLoopRef               theRunLoop = CFRunLoopGetCurrent();
    AudioObjectPropertyAddress property   = { kAudioHardwarePropertyRunLoop,
                                              kAudioObjectPropertyScopeGlobal,
                                              kAudioObjectPropertyElementMaster };

    AudioObjectSetPropertyData ( kAudioObjectSystemObject,
                                 &property,
                                 0,
                                 NULL,
                                 sizeof ( CFRunLoopRef ),
                                 &theRunLoop );


    // Get available input/output devices --------------------------------------
    UInt32                     iPropertySize = 0;
    AudioObjectPropertyAddress stPropertyAddress;

    stPropertyAddress.mScope   = kAudioObjectPropertyScopeGlobal;
    stPropertyAddress.mElement = kAudioObjectPropertyElementMaster;

    // first get property size of devices array and allocate memory
    stPropertyAddress.mSelector = kAudioHardwarePropertyDevices;

    AudioObjectGetPropertyDataSize ( kAudioObjectSystemObject,
                                     &stPropertyAddress,
                                     0,
                                     NULL,
                                     &iPropertySize );

    CVector<AudioDeviceID> vAudioDevices ( iPropertySize );

    // now actually query all devices present in the system
    AudioObjectGetPropertyData ( kAudioObjectSystemObject,
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 &iPropertySize,
                                 &vAudioDevices[0] );

    // calculate device count based on size of returned data array
    const UInt32 iDeviceCount = iPropertySize / sizeof ( AudioDeviceID );

    // always add system default devices for input and output as first entry
    lNumDevs                 = 0;
    strDriverNames[lNumDevs] = "System Default In/Out Devices";

    iPropertySize               = sizeof ( AudioDeviceID );
    stPropertyAddress.mSelector = kAudioHardwarePropertyDefaultInputDevice;

    if ( AudioObjectGetPropertyData ( kAudioObjectSystemObject,
                                      &stPropertyAddress,
                                      0,
                                      NULL,
                                      &iPropertySize,
                                      &audioInputDevice[lNumDevs] ) )
    {
        throw CGenErr ( tr ( "CoreAudio input AudioHardwareGetProperty call failed. "
                             "It seems that no sound card is available in the system." ) );
    }

    iPropertySize               = sizeof ( AudioDeviceID );
    stPropertyAddress.mSelector = kAudioHardwarePropertyDefaultOutputDevice;

    if ( AudioObjectGetPropertyData ( kAudioObjectSystemObject,
                                      &stPropertyAddress,
                                      0,
                                      NULL,
                                      &iPropertySize,
                                      &audioOutputDevice[lNumDevs] ) )
    {
        throw CGenErr ( tr ( "CoreAudio output AudioHardwareGetProperty call failed. "
                             "It seems that no sound card is available in the system." ) );
    }

    lNumDevs++; // next device

    // add detected devices
    //
    // we add combined entries for input and output for each device so that we
    // do not need two combo boxes in the GUI for input and output (therefore
    // all possible combinations are required which can be a large number)
    for ( UInt32 i = 0; i < iDeviceCount; i++ )
    {
        for ( UInt32 j = 0; j < iDeviceCount; j++ )
        {
            // get device infos for both current devices
            QString strDeviceName_i;
            QString strDeviceName_j;
            bool    bIsInput_i;
            bool    bIsInput_j;
            bool    bIsOutput_i;
            bool    bIsOutput_j;

            GetAudioDeviceInfos ( vAudioDevices[i],
                                  strDeviceName_i,
                                  bIsInput_i,
                                  bIsOutput_i );

            GetAudioDeviceInfos ( vAudioDevices[j],
                                  strDeviceName_j,
                                  bIsInput_j,
                                  bIsOutput_j );

            // check if i device is input and j device is output and that we are
            // in range
            if ( bIsInput_i && bIsOutput_j && ( lNumDevs < MAX_NUMBER_SOUND_CARDS ) )
            {
                strDriverNames[lNumDevs] = "in: " +
                    strDeviceName_i + "/out: " +
                    strDeviceName_j;

                // store audio device IDs
                audioInputDevice[lNumDevs]  = vAudioDevices[i];
                audioOutputDevice[lNumDevs] = vAudioDevices[j];

                lNumDevs++; // next device
            }
        }
    }

    // init device index as not initialized (invalid)
    lCurDev                    = INVALID_SNC_CARD_DEVICE;
    CurrentAudioInputDeviceID  = 0;
    CurrentAudioOutputDeviceID = 0;
    iNumInChan                 = 0;
    iNumOutChan                = 0;
    iSelInputLeftChannel       = 0;
    iSelInputRightChannel      = 0;
    iSelOutputLeftChannel      = 0;
    iSelOutputRightChannel     = 0;


    // Optional MIDI initialization --------------------------------------------
    if ( iCtrlMIDIChannel != INVALID_MIDI_CH )
    {
        // create client and ports
        MIDIClientRef midiClient = static_cast<MIDIClientRef> ( NULL );
        MIDIClientCreate ( CFSTR ( APP_NAME ), NULL, NULL, &midiClient );
        MIDIInputPortCreate ( midiClient, CFSTR ( "Input port" ), callbackMIDI, this, &midiInPortRef );

        // open connections from all sources
        const int iNMIDISources = MIDIGetNumberOfSources();

        for ( int i = 0; i < iNMIDISources; i++ )
        {
            MIDIEndpointRef src = MIDIGetSource ( i );
            MIDIPortConnectSource ( midiInPortRef, src, NULL ) ;
        }
    }
}

void CSound::GetAudioDeviceInfos ( const AudioDeviceID DeviceID,
                                   QString&            strDeviceName,
                                   bool&               bIsInput,
                                   bool&               bIsOutput )
{
    UInt32                     iPropertySize;
    AudioObjectPropertyAddress stPropertyAddress;

    // init return values
    bIsInput  = false;
    bIsOutput = false;

    // check if device is input or output or both (is that possible?)
    stPropertyAddress.mSelector = kAudioDevicePropertyStreams;
    stPropertyAddress.mElement  = kAudioObjectPropertyElementMaster;

    // input check
    iPropertySize            = 0;
    stPropertyAddress.mScope = kAudioDevicePropertyScopeInput;

    AudioObjectGetPropertyDataSize ( DeviceID,
                                     &stPropertyAddress,
                                     0,
                                     NULL,
                                     &iPropertySize );

    bIsInput = ( iPropertySize > 0 ); // check if any input streams are available

    // output check
    iPropertySize            = 0;
    stPropertyAddress.mScope = kAudioDevicePropertyScopeOutput;

    AudioObjectGetPropertyDataSize ( DeviceID,
                                     &stPropertyAddress,
                                     0,
                                     NULL,
                                     &iPropertySize );

    bIsOutput = ( iPropertySize > 0 ); // check if any output streams are available

    // get property name
    CFStringRef sPropertyStringValue = NULL;

    stPropertyAddress.mSelector = kAudioObjectPropertyName;
    stPropertyAddress.mScope    = kAudioObjectPropertyScopeGlobal;
    iPropertySize               = sizeof ( CFStringRef );

    AudioObjectGetPropertyData ( DeviceID,
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 &iPropertySize,
                                 &sPropertyStringValue );

    // convert string
    if ( !ConvertCFStringToQString ( sPropertyStringValue, strDeviceName ) )
    {
        // use a default name in case the conversion did not succeed
        strDeviceName = "UNKNOWN";
    }
}

int CSound::CountChannels ( AudioDeviceID devID,
                            bool          isInput )
{
    OSStatus err;
    UInt32   propSize;
    int      result = 0;

    AudioObjectPropertyScope theScope = isInput ? kAudioDevicePropertyScopeInput : kAudioDevicePropertyScopeOutput;

    AudioObjectPropertyAddress theAddress = { kAudioDevicePropertyStreamConfiguration,
                                              theScope,
                                              0 };

    AudioObjectGetPropertyDataSize ( devID, &theAddress, 0, NULL, &propSize );

    AudioBufferList *buflist = (AudioBufferList*) malloc ( propSize );

    err = AudioObjectGetPropertyData ( devID, &theAddress, 0, NULL, &propSize, buflist );

    if ( !err )
    {
        for ( UInt32 i = 0; i < buflist->mNumberBuffers; ++i )
        {
            // The correct value mNumberChannels for an AudioBuffer can be derived from the mChannelsPerFrame
            // and the interleaved flag. For non interleaved formats, mNumberChannels is always 1.
            // For interleaved formats, mNumberChannels is equal to mChannelsPerFrame.
            result += buflist->mBuffers[i].mNumberChannels;
        }
    }
    free ( buflist );

    return result;
}

QString CSound::LoadAndInitializeDriver ( int iDriverIdx )
{
    // check device capabilities if it fullfills our requirements
    const QString strStat = CheckDeviceCapabilities ( iDriverIdx );

    // check if device is capable
    if ( strStat.isEmpty() )
    {
        // store ID of selected driver if initialization was successful
        lCurDev                    = iDriverIdx;
        CurrentAudioInputDeviceID  = audioInputDevice[iDriverIdx];
        CurrentAudioOutputDeviceID = audioOutputDevice[iDriverIdx];

        // the device has changed, per definition we reset the channel
        // mapping to the defaults (first two available channels)
        iSelInputLeftChannel   = 0;
        iSelInputRightChannel  = min ( iNumInChan - 1, 1 );
        iSelOutputLeftChannel  = 0;
        iSelOutputRightChannel = min ( iNumOutChan - 1, 1 );
    }

    return strStat;
}

QString CSound::CheckDeviceCapabilities ( const int iDriverIdx )
{
    UInt32                      iPropertySize;
    AudioStreamBasicDescription	CurDevStreamFormat;
    Float64                     inputSampleRate   = 0;
    Float64                     outputSampleRate  = 0;
    const Float64               fSystemSampleRate = static_cast<Float64> ( SYSTEM_SAMPLE_RATE_HZ );
    AudioObjectPropertyAddress  stPropertyAddress;

    stPropertyAddress.mScope    = kAudioObjectPropertyScopeGlobal;
    stPropertyAddress.mElement  = kAudioObjectPropertyElementMaster;

    // check input device sample rate
    stPropertyAddress.mSelector = kAudioDevicePropertyNominalSampleRate;
    iPropertySize               = sizeof ( Float64 );

    AudioObjectGetPropertyData ( audioInputDevice[iDriverIdx],
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 &iPropertySize,
                                 &inputSampleRate );

    if ( inputSampleRate != fSystemSampleRate )
    {
        // try to change the sample rate
        if ( AudioObjectSetPropertyData ( audioInputDevice[iDriverIdx],
                                          &stPropertyAddress,
                                          0,
                                          NULL,
                                          sizeof ( Float64 ),
                                          &fSystemSampleRate ) != noErr )
        {
            return QString ( tr ( "Current system audio input device sample "
                "rate of %1 Hz is not supported. Please open the Audio-MIDI-Setup in "
                "Applications->Utilities and try to set a sample rate of %2 Hz." ) ).arg (
                static_cast<int> ( inputSampleRate ) ).arg ( SYSTEM_SAMPLE_RATE_HZ );
        }
    }

    // check output device sample rate
    iPropertySize = sizeof ( Float64 );

    AudioObjectGetPropertyData ( audioOutputDevice[iDriverIdx],
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 &iPropertySize,
                                 &outputSampleRate );

    if ( outputSampleRate != fSystemSampleRate )
    {
        // try to change the sample rate
        if ( AudioObjectSetPropertyData ( audioOutputDevice[iDriverIdx],
                                          &stPropertyAddress,
                                          0,
                                          NULL,
                                          sizeof ( Float64 ),
                                          &fSystemSampleRate ) != noErr )
        {
            return QString ( tr ( "Current system audio output device sample "
                "rate of %1 Hz is not supported. Please open the Audio-MIDI-Setup in "
                "Applications->Utilities and try to set a sample rate of %2 Hz." ) ).arg (
                static_cast<int> ( outputSampleRate ) ).arg ( SYSTEM_SAMPLE_RATE_HZ );
        }
    }

    // get the stream ID of the input device (at least one stream must always exist)
    iPropertySize               = 0;
    stPropertyAddress.mSelector = kAudioDevicePropertyStreams;
    stPropertyAddress.mScope    = kAudioObjectPropertyScopeInput;

    AudioObjectGetPropertyDataSize ( audioInputDevice[iDriverIdx],
                                     &stPropertyAddress,
                                     0,
                                     NULL,
                                     &iPropertySize );

    CVector<AudioStreamID> vInputStreamIDList ( iPropertySize );

    AudioObjectGetPropertyData ( audioInputDevice[iDriverIdx],
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 &iPropertySize,
                                 &vInputStreamIDList[0] );

    const AudioStreamID inputStreamID = vInputStreamIDList[0];

    // get the stream ID of the output device (at least one stream must always exist)
    iPropertySize               = 0;
    stPropertyAddress.mSelector = kAudioDevicePropertyStreams;
    stPropertyAddress.mScope    = kAudioObjectPropertyScopeOutput;

    AudioObjectGetPropertyDataSize ( audioOutputDevice[iDriverIdx],
                                     &stPropertyAddress,
                                     0,
                                     NULL,
                                     &iPropertySize );

    CVector<AudioStreamID> vOutputStreamIDList ( iPropertySize );

    AudioObjectGetPropertyData ( audioOutputDevice[iDriverIdx],
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 &iPropertySize,
                                 &vOutputStreamIDList[0] );

    const AudioStreamID outputStreamID = vOutputStreamIDList[0];

    // According to the AudioHardware documentation: "If the format is a linear PCM
    // format, the data will always be presented as 32 bit, native endian floating
    // point. All conversions to and from the true physical format of the hardware
    // is handled by the devices driver.".
    // check the input
    iPropertySize               = sizeof ( AudioStreamBasicDescription );
    stPropertyAddress.mSelector = kAudioStreamPropertyVirtualFormat;
    stPropertyAddress.mScope    = kAudioObjectPropertyScopeGlobal;

    AudioObjectGetPropertyData ( inputStreamID,
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 &iPropertySize,
                                 &CurDevStreamFormat );

    if ( ( CurDevStreamFormat.mFormatID        != kAudioFormatLinearPCM ) ||
         ( CurDevStreamFormat.mFramesPerPacket != 1 ) ||
         ( CurDevStreamFormat.mBitsPerChannel  != 32 ) ||
         ( !( CurDevStreamFormat.mFormatFlags & kAudioFormatFlagIsFloat ) ) ||
         ( !( CurDevStreamFormat.mFormatFlags & kAudioFormatFlagIsPacked ) ) )
    {
        return tr ( "The audio input stream format for this audio device is "
                    "not compatible with this software." );
    }

    // check the output
    AudioObjectGetPropertyData ( outputStreamID,
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 &iPropertySize,
                                 &CurDevStreamFormat );

    if ( ( CurDevStreamFormat.mFormatID        != kAudioFormatLinearPCM ) ||
         ( CurDevStreamFormat.mFramesPerPacket != 1 ) ||
         ( CurDevStreamFormat.mBitsPerChannel  != 32 ) ||
         ( !( CurDevStreamFormat.mFormatFlags & kAudioFormatFlagIsFloat ) ) ||
         ( !( CurDevStreamFormat.mFormatFlags & kAudioFormatFlagIsPacked ) ) )
    {
        return tr ( "The audio output stream format for this audio device is "
                    "not compatible with this software." );
    }

    // store the input and out number of channels for this device
    iNumInChan  = CountChannels ( audioInputDevice[iDriverIdx], true );
    iNumOutChan = CountChannels ( audioOutputDevice[iDriverIdx], false );

    // clip the number of input/output channels to our allowed maximum
    if ( iNumInChan > MAX_NUM_IN_OUT_CHANNELS )
    {
        iNumInChan = MAX_NUM_IN_OUT_CHANNELS;
    }
    if ( iNumOutChan > MAX_NUM_IN_OUT_CHANNELS )
    {
        iNumOutChan = MAX_NUM_IN_OUT_CHANNELS;
    }

    // get the channel names of the input device
    for ( int iCurInCH = 0; iCurInCH < iNumInChan; iCurInCH++ )
    {
        CFStringRef sPropertyStringValue = NULL;

        stPropertyAddress.mSelector = kAudioObjectPropertyElementName;
        stPropertyAddress.mElement  = iCurInCH + 1;
        stPropertyAddress.mScope    = kAudioObjectPropertyScopeInput;
        iPropertySize               = sizeof ( CFStringRef );

        AudioObjectGetPropertyData ( audioInputDevice[iDriverIdx],
                                     &stPropertyAddress,
                                     0,
                                     NULL,
                                     &iPropertySize,
                                     &sPropertyStringValue );

        // convert string
        const bool bConvOK = ConvertCFStringToQString ( sPropertyStringValue,
                                                        sChannelNamesInput[iCurInCH] );

        // add the "[n]:" at the beginning as is in the Audio-Midi-Setup
        if ( !bConvOK || ( iPropertySize == 0 ) )
        {
            // use a default name in case there was an error or the name is empty
            sChannelNamesInput[iCurInCH] =
                QString ( "%1: Channel %1" ).arg ( iCurInCH + 1 );
        }
        else
        {
            sChannelNamesInput[iCurInCH].prepend ( QString ( "%1: " ).arg ( iCurInCH + 1 ) );
        }
    }

    // get the channel names of the output device
    for ( int iCurOutCH = 0; iCurOutCH < iNumOutChan; iCurOutCH++ )
    {
        CFStringRef sPropertyStringValue = NULL;

        stPropertyAddress.mSelector = kAudioObjectPropertyElementName;
        stPropertyAddress.mElement  = iCurOutCH + 1;
        stPropertyAddress.mScope    = kAudioObjectPropertyScopeOutput;
        iPropertySize               = sizeof ( CFStringRef );

        AudioObjectGetPropertyData ( audioOutputDevice[iDriverIdx],
                                     &stPropertyAddress,
                                     0,
                                     NULL,
                                     &iPropertySize,
                                     &sPropertyStringValue );

        // convert string
        const bool bConvOK = ConvertCFStringToQString ( sPropertyStringValue,
                                                        sChannelNamesOutput[iCurOutCH] );

        // add the "[n]:" at the beginning as is in the Audio-Midi-Setup
        if ( !bConvOK || ( iPropertySize == 0 ) )
        {
            // use a defalut name in case there was an error or the name is empty
            sChannelNamesOutput[iCurOutCH] =
                QString ( "%1: Channel %1" ).arg ( iCurOutCH + 1 );
        }
        else
        {
            sChannelNamesOutput[iCurOutCH].prepend ( QString ( "%1: " ).arg ( iCurOutCH + 1 ) );
        }
    }

    // everything is ok, return empty string for "no error" case
    return "";
}

void CSound::SetLeftInputChannel  ( const int iNewChan )
{
    // apply parameter after input parameter check
    if ( ( iNewChan >= 0 ) && ( iNewChan < iNumInChan ) )
    {
        iSelInputLeftChannel = iNewChan;
    }
}

void CSound::SetRightInputChannel ( const int iNewChan )
{
    // apply parameter after input parameter check
    if ( ( iNewChan >= 0 ) && ( iNewChan < iNumInChan ) )
    {
        iSelInputRightChannel = iNewChan;
    }
}

void CSound::SetLeftOutputChannel  ( const int iNewChan )
{
    // apply parameter after input parameter check
    if ( ( iNewChan >= 0 ) && ( iNewChan < iNumOutChan ) )
    {
        iSelOutputLeftChannel = iNewChan;
    }
}

void CSound::SetRightOutputChannel ( const int iNewChan )
{
    // apply parameter after input parameter check
    if ( ( iNewChan >= 0 ) && ( iNewChan < iNumOutChan ) )
    {
        iSelOutputRightChannel = iNewChan;
    }
}

void CSound::Start()
{
    if(!IsCARunning()) {
        OSStatus err = SetupAUHAL(audioInputDevice[lCurDev]);
        verify_noerr(err);
        
        // Setup Graph containing Varispeed Unit & Default Output Unit
        err = SetupGraph(audioOutputDevice[lCurDev]);
        verify_noerr(err);
                
        // the varispeed unit should only be conected after the input and output formats have been set
        err = AUGraphConnectNodeInput(mGraph, mVarispeedNode, 0, mOutputNode, 0);
        verify_noerr(err);
        
        err = AUGraphInitialize(mGraph);
        verify_noerr(err);
        
        //Start pulling for audio data
        err = AudioOutputUnitStart(mInputUnit);
        verify_noerr(err);
        
        err = AUGraphStart(mGraph);
        verify_noerr(err);
        
        //reset sample times
        mFirstInputTime = -1;
        mFirstOutputTime = -1;
        
        bRun = true;
    
    AudioObjectPropertyAddress stPropertyAddress;

    stPropertyAddress.mElement  = kAudioObjectPropertyElementMaster;
    stPropertyAddress.mScope    = kAudioObjectPropertyScopeGlobal;

    // setup callback for xruns (only for input is enough)
    stPropertyAddress.mSelector = kAudioDeviceProcessorOverload;

    AudioObjectAddPropertyListener ( audioInputDevice[lCurDev],
                                     &stPropertyAddress,
                                     deviceNotification,
                                     this );

    // setup callbacks for device property changes
    stPropertyAddress.mSelector = kAudioDevicePropertyDeviceHasChanged;

    AudioObjectAddPropertyListener ( audioInputDevice[lCurDev],
                                     &stPropertyAddress,
                                     deviceNotification,
                                     this );

    AudioObjectAddPropertyListener ( audioOutputDevice[lCurDev],
                                     &stPropertyAddress,
                                     deviceNotification,
                                     this );
//
//    // register the callback function for input and output
//    AudioDeviceCreateIOProcID ( audioInputDevice[lCurDev],
//                                callbackIO,
//                                this,
//                                &audioInputProcID );
//
//    AudioDeviceCreateIOProcID ( audioOutputDevice[lCurDev],
//                                callbackIO,
//                                this,
//                                &audioOutputProcID );
//
//    // start the audio stream
//    AudioDeviceStart ( audioInputDevice[lCurDev], audioInputProcID );
//    AudioDeviceStart ( audioOutputDevice[lCurDev], audioOutputProcID );
//
        // call base class
        CSoundBase::Start();
    }
}

OSStatus CSound::SetupAUHAL(AudioDeviceID in)
{
    OSStatus err = noErr;
    
    AudioComponent comp;
    AudioComponentDescription desc;
    
    //There are several different types of Audio Units.
    //Some audio units serve as Outputs, Mixers, or DSP
    //units. See AUComponent.h for listing
    desc.componentType = kAudioUnitType_Output;
    
    //Every Component has a subType, which will give a clearer picture
    //of what this components function will be.
    desc.componentSubType = kAudioUnitSubType_HALOutput;
    
    //all Audio Units in AUComponent.h must use
    //"kAudioUnitManufacturer_Apple" as the Manufacturer
    desc.componentManufacturer = kAudioUnitManufacturer_Apple;
    desc.componentFlags = 0;
    desc.componentFlagsMask = 0;
    
    //Finds a component that meets the desc spec's
    comp = AudioComponentFindNext(NULL, &desc);
    if (comp == NULL) exit (-1);
    
    //gains access to the services provided by the component
    err = AudioComponentInstanceNew(comp, &mInputUnit);
    checkErr(err);
    
    //AUHAL needs to be initialized before anything is done to it
    err = AudioUnitInitialize(mInputUnit);
    checkErr(err);
    
    err = EnableIO();
    checkErr(err);
    
    err= SetInputDeviceAsCurrent(in);
    checkErr(err);
    
    err = CallbackSetup();
    checkErr(err);
    
    //Don't setup buffers until you know what the
    //input and output device audio streams look like.
    
    err = AudioUnitInitialize(mInputUnit);
    
    return err;
}

OSStatus CSound::EnableIO()
{
    OSStatus err = noErr;
    UInt32 enableIO;
    
    ///////////////
    //ENABLE IO (INPUT)
    //You must enable the Audio Unit (AUHAL) for input and disable output
    //BEFORE setting the AUHAL's current device.
    
    //Enable input on the AUHAL
    enableIO = 1;
    err =  AudioUnitSetProperty(mInputUnit,
                                kAudioOutputUnitProperty_EnableIO,
                                kAudioUnitScope_Input,
                                1, // input element
                                &enableIO,
                                sizeof(enableIO));
    checkErr(err);
    
    //disable Output on the AUHAL
    enableIO = 0;
    err = AudioUnitSetProperty(mInputUnit,
                              kAudioOutputUnitProperty_EnableIO,
                              kAudioUnitScope_Output,
                              0,   //output element
                              &enableIO,
                              sizeof(enableIO));
    return err;
}
OSStatus CSound::CallbackSetup()
{
    OSStatus err = noErr;
    AURenderCallbackStruct input;
    
    input.inputProc = InputProc;
    input.inputProcRefCon = this;
    
    //Setup the input callback.
    err = AudioUnitSetProperty(mInputUnit,
                              kAudioOutputUnitProperty_SetInputCallback,
                              kAudioUnitScope_Global,
                              0,
                              &input,
                              sizeof(input));
    checkErr(err);
    return err;
}

OSStatus CSound::SetInputDeviceAsCurrent(AudioDeviceID in)
{
    UInt32 size = sizeof(AudioDeviceID);
    OSStatus err = noErr;
    
    AudioObjectPropertyAddress theAddress = { kAudioHardwarePropertyDefaultInputDevice,
                                              kAudioObjectPropertyScopeGlobal,
                                              kAudioObjectPropertyElementMaster };
    
    if(in == kAudioDeviceUnknown) //get the default input device if device is unknown
    {
        err = AudioObjectGetPropertyData(kAudioObjectSystemObject, &theAddress, 0, NULL, &size, &in);
        checkErr(err);
    }
    mInputDevice.Init(in, true);
    
    //Set the Current Device to the AUHAL.
    //this should be done only after IO has been enabled on the AUHAL.
    err = AudioUnitSetProperty(mInputUnit,
                              kAudioOutputUnitProperty_CurrentDevice,
                              kAudioUnitScope_Global,
                              0,
                              &mInputDevice.mID,
                              sizeof(mInputDevice.mID));
    checkErr(err);
    return err;
}

OSStatus CSound::SetupGraph(AudioDeviceID out)
{
    OSStatus err = noErr;
    AURenderCallbackStruct output;
    
    //Make a New Graph
    err = NewAUGraph(&mGraph);
    checkErr(err);

    //Open the Graph, AudioUnits are opened but not initialized
    err = AUGraphOpen(mGraph);
    checkErr(err);
    
    err = MakeGraph();
    checkErr(err);
        
    err = SetOutputDeviceAsCurrent(out);
    checkErr(err);
    
    //Tell the output unit not to reset timestamps
    //Otherwise sample rate changes will cause sync los
    UInt32 startAtZero = 0;
    err = AudioUnitSetProperty(mOutputUnit,
                              kAudioOutputUnitProperty_StartTimestampsAtZero,
                              kAudioUnitScope_Global,
                              0,
                              &startAtZero,
                              sizeof(startAtZero));
    checkErr(err);
    
    output.inputProc = OutputProc;
    output.inputProcRefCon = this;
    
    err = AudioUnitSetProperty(mVarispeedUnit,
                              kAudioUnitProperty_SetRenderCallback,
                              kAudioUnitScope_Input,
                              0,
                              &output,
                              sizeof(output));
    checkErr(err);
    
    return err;
}


void CSound::Stop()
{
    OSStatus err = noErr;
    if(IsRunning()) {
        
        //Stop the AUHAL
        err = AudioOutputUnitStop(mInputUnit);
        verify_noerr(err);
        
        err = AUGraphStop(mGraph);
        verify_noerr(err);
        mFirstInputTime = -1;
        mFirstOutputTime = -1;
        bRun = false;
        
        if(mInputBuffer) {
            for(UInt32 i = 0; i<mInputBuffer->mNumberBuffers; i++)
                free(mInputBuffer->mBuffers[i].mData);
            free(mInputBuffer);
            mInputBuffer = 0;
        }
        
        AudioUnitUninitialize(mInputUnit);
        AUGraphClose(mGraph);
        DisposeAUGraph(mGraph);
        AudioComponentInstanceDispose(mInputUnit);
        
    
    
//    // stop the audio stream
//    AudioDeviceStop ( audioInputDevice[lCurDev], audioInputProcID );
//    AudioDeviceStop ( audioOutputDevice[lCurDev], audioOutputProcID );
//
//    // unregister the callback function for input and output
//    AudioDeviceDestroyIOProcID ( audioInputDevice[lCurDev], audioInputProcID );
//    AudioDeviceDestroyIOProcID ( audioOutputDevice[lCurDev], audioOutputProcID );
//
    AudioObjectPropertyAddress stPropertyAddress;

    stPropertyAddress.mElement  = kAudioObjectPropertyElementMaster;
    stPropertyAddress.mScope    = kAudioObjectPropertyScopeGlobal;

    // unregister callback functions for device property changes
    stPropertyAddress.mSelector = kAudioDevicePropertyDeviceHasChanged;

    AudioObjectRemovePropertyListener( audioOutputDevice[lCurDev],
                                       &stPropertyAddress,
                                       deviceNotification,
                                       this );

    AudioObjectRemovePropertyListener( audioInputDevice[lCurDev],
                                       &stPropertyAddress,
                                       deviceNotification,
                                       this );

    // unregister the callback function for xruns
    stPropertyAddress.mSelector = kAudioDeviceProcessorOverload;

    AudioObjectRemovePropertyListener( audioInputDevice[lCurDev],
                                       &stPropertyAddress,
                                       deviceNotification,
                                       this );

        // call base class
        CSoundBase::Stop();
    }
}





bool CSound::IsCARunning()
{
    OSStatus err = noErr;
    UInt32 auhalRunning = 0, size = 0;
    Boolean graphRunning = false;
    size = sizeof(auhalRunning);
    if(mInputUnit)
    {
        err = AudioUnitGetProperty(mInputUnit,
                                kAudioOutputUnitProperty_IsRunning,
                                kAudioUnitScope_Global,
                                0, // input element
                                &auhalRunning,
                                &size);
        checkErr(err);
    }
    
    if(mGraph) {
        err = AUGraphIsRunning(mGraph,&graphRunning);
        checkErr(err);
    }
    
    bRun = (auhalRunning || graphRunning);
    return CSoundBase::IsRunning();
}


int CSound::Init ( const int iNewPrefMonoBufferSize )
{
    UInt32 iActualMonoBufferSize;

    // Error message string: in case buffer sizes on input and output cannot be
    // set to the same value
    const QString strErrBufSize = tr ( "The buffer sizes of the current "
        "input and output audio device cannot be set to a common value. Please "
        "choose other input/output audio devices in your system settings." );

    // try to set input buffer size
    iActualMonoBufferSize =
        SetBufferSize ( audioInputDevice[lCurDev], true, iNewPrefMonoBufferSize );

    if ( iActualMonoBufferSize != static_cast<UInt32> ( iNewPrefMonoBufferSize ) )
    {
        // try to set the input buffer size to the output so that we
        // have a matching pair
        if ( SetBufferSize ( audioOutputDevice[lCurDev], false, iActualMonoBufferSize ) !=
             iActualMonoBufferSize )
        {
            throw CGenErr ( strErrBufSize );
        }
    }
    else
    {
        // try to set output buffer size
        if ( SetBufferSize ( audioOutputDevice[lCurDev], false, iNewPrefMonoBufferSize ) !=
             static_cast<UInt32> ( iNewPrefMonoBufferSize ) )
        {
            throw CGenErr ( strErrBufSize );
        }
    }

    // store buffer size
    iCoreAudioBufferSizeMono = iActualMonoBufferSize;

    // init base class
    CSoundBase::Init ( iCoreAudioBufferSizeMono );

    // set internal buffer size value and calculate stereo buffer size
    iCoreAudioBufferSizeStereo = 2 * iCoreAudioBufferSizeMono;

    // create memory for intermediate audio buffer
    vecsTmpAudioSndCrdStereo.Init ( iCoreAudioBufferSizeStereo );

    return iCoreAudioBufferSizeMono;
}

UInt32 CSound::SetBufferSize ( AudioDeviceID& audioDeviceID,
                               const bool     bIsInput,
                               UInt32         iPrefBufferSize )
{
    AudioObjectPropertyAddress stPropertyAddress;
    stPropertyAddress.mSelector = kAudioDevicePropertyBufferFrameSize;

    if ( bIsInput )
    {
        stPropertyAddress.mScope = kAudioDevicePropertyScopeInput;
    }
    else
    {
        stPropertyAddress.mScope = kAudioDevicePropertyScopeOutput;
    }

    stPropertyAddress.mElement = kAudioObjectPropertyElementMaster;

    // first set the value
    UInt32 iSizeBufValue = sizeof ( UInt32 );

    AudioObjectSetPropertyData ( audioDeviceID,
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 iSizeBufValue,
                                 &iPrefBufferSize );

    // read back which value is actually used
    UInt32 iActualMonoBufferSize = 0;

    AudioObjectGetPropertyData ( audioDeviceID,
                                 &stPropertyAddress,
                                 0,
                                 NULL,
                                 &iSizeBufValue,
                                 &iActualMonoBufferSize );

    return iActualMonoBufferSize;
}

OSStatus CSound::deviceNotification ( AudioDeviceID,
                                      UInt32,
                                      const AudioObjectPropertyAddress* inAddresses,
                                      void*                             inRefCon )
{
    CSound* pSound = static_cast<CSound*> ( inRefCon );

    if ( inAddresses->mSelector == kAudioDevicePropertyDeviceHasChanged )
    {
        // if any property of the device has changed, do a full reload
        pSound->EmitReinitRequestSignal ( RS_RELOAD_RESTART_AND_INIT );
    }

/*
    if ( inAddresses->mSelector == kAudioDeviceProcessorOverload )
    {
        // xrun handling (it is important to act on xruns under CoreAudio
        // since it seems that the xrun situation stays stable for a
        // while and would give you a long time bad audio)
        pSound->EmitReinitRequestSignal ( RS_ONLY_RESTART );
    }
*/

    return noErr;
}

OSStatus CSound::InputProc(void *inRefCon,
                           AudioUnitRenderActionFlags *ioActionFlags,
                           const AudioTimeStamp *inTimeStamp,
                           UInt32 inBusNumber,
                           UInt32 inNumberFrames,
                           AudioBufferList * ioData)
{
    OSStatus err = noErr;
    
    CSound *This = (CSound *)inRefCon;
    if (This->mFirstInputTime < 0.)
        This->mFirstInputTime = inTimeStamp->mSampleTime;
        
    //Get the new audio data
    err = AudioUnitRender(This->mInputUnit,
                         ioActionFlags,
                         inTimeStamp,
                         inBusNumber,
                         inNumberFrames, //# of frames requested
                         This->mInputBuffer);// Audio Buffer List to hold data
    checkErr(err);
        
    if(!err) {
        callbackIO(This->audioInputDevice[This->lCurDev], inTimeStamp, ioData, inTimeStamp, nil, nil, inRefCon);
    }

    return err;
}

inline void MakeBufferSilent (AudioBufferList * ioData)
{
    for(UInt32 i=0; i<ioData->mNumberBuffers;i++)
        memset(ioData->mBuffers[i].mData, 0, ioData->mBuffers[i].mDataByteSize);
}

void CSound::ComputeThruOffset()
{
    //The initial latency will at least be the saftey offset's of the devices + the buffer sizes
    mInToOutSampleOffset = SInt32(mInputDevice.mSafetyOffset +  mInputDevice.mBufferSizeFrames +
                        mOutputDevice.mSafetyOffset + mOutputDevice.mBufferSizeFrames);
}

OSStatus CSound::OutputProc(void *inRefCon,
                            AudioUnitRenderActionFlags *ioActionFlags,
                            const AudioTimeStamp *TimeStamp,
                            UInt32 inBusNumber,
                            UInt32 inNumberFrames,
                            AudioBufferList * ioData)
{
    OSStatus err = noErr;
    CSound *This = (CSound *)inRefCon;
    Float64 rate = 0.0;
    AudioTimeStamp inTS, outTS;
        
    if (This->mFirstInputTime < 0.) {
        // input hasn't run yet -> silence
        MakeBufferSilent (ioData);
        return noErr;
    }
    
    //use the varispeed playback rate to offset small discrepancies in sample rate
    //first find the rate scalars of the input and output devices
    err = AudioDeviceGetCurrentTime(This->mInputDevice.mID, &inTS);
    // this callback may still be called a few times after the device has been stopped
    if (err)
    {
        MakeBufferSilent (ioData);
        return noErr;
    }
        
    err = AudioDeviceGetCurrentTime(This->mOutputDevice.mID, &outTS);
    checkErr(err);
    
    rate = inTS.mRateScalar / outTS.mRateScalar;
    err = AudioUnitSetParameter(This->mVarispeedUnit,kVarispeedParam_PlaybackRate,kAudioUnitScope_Global,0, rate,0);
    checkErr(err);
    
    //get Delta between the devices and add it to the offset
    if (This->mFirstOutputTime < 0.) {
        This->mFirstOutputTime = TimeStamp->mSampleTime;
        Float64 delta = (This->mFirstInputTime - This->mFirstOutputTime);
        This->ComputeThruOffset();

        if (delta < 0.0)
            This->mInToOutSampleOffset -= delta;
        else
            This->mInToOutSampleOffset = -delta + This->mInToOutSampleOffset;
                    
        MakeBufferSilent (ioData);
        return noErr;
    }

    if (!err) {
    //copy the data from the buffers
        return CSound::callbackIO(This->CurrentAudioOutputDeviceID, nil, nil, TimeStamp, ioData, TimeStamp, inRefCon);
    }
    return err;
}

OSStatus CSound::callbackIO ( AudioDeviceID          inDevice,
                              const AudioTimeStamp*,
                              const AudioBufferList* inInputData,
                              const AudioTimeStamp*,
                              AudioBufferList*       outOutputData,
                              const AudioTimeStamp*,
                              void*                  inRefCon )
{
    CSound* pSound = static_cast<CSound*> ( inRefCon );

    // both, the input and output device use the same callback function
    QMutexLocker locker ( &pSound->Mutex );

    const int iCoreAudioBufferSizeMono = pSound->iCoreAudioBufferSizeMono;
    const int iNumInChan               = pSound->iNumInChan;
    const int iNumOutChan              = pSound->iNumOutChan;
    const int iSelInputLeftChannel     = pSound->iSelInputLeftChannel;
    const int iSelInputRightChannel    = pSound->iSelInputRightChannel;
    const int iSelOutputLeftChannel    = pSound->iSelOutputLeftChannel;
    const int iSelOutputRightChannel   = pSound->iSelOutputRightChannel;

    if ( inDevice == pSound->CurrentAudioInputDeviceID )
    {
        // check size (float32 has four bytes)
        if ( inInputData && inInputData->mBuffers[0].mDataByteSize ==
             static_cast<UInt32> ( iCoreAudioBufferSizeMono * iNumInChan * 4 ) )
        {
            // one buffer with all the channels in interleaved format:
            // get a pointer to the input data of the correct type
            Float32* pInData = static_cast<Float32*> ( inInputData->mBuffers[0].mData );

            // copy input data
            for ( int i = 0; i < iCoreAudioBufferSizeMono; i++ )
            {
                // left
                pSound->vecsTmpAudioSndCrdStereo[2 * i] =
                    (short) ( pInData[iNumInChan * i + iSelInputLeftChannel] * _MAXSHORT );

                // right
                pSound->vecsTmpAudioSndCrdStereo[2 * i + 1] =
                    (short) ( pInData[iNumInChan * i + iSelInputRightChannel] * _MAXSHORT );

/*
// TEST mix channel with micro to the stereo output
if ( iNumInChan == 4 )
{
    // add mic input on input channel 4 to both stereo channels
    pSound->vecsTmpAudioSndCrdStereo[2 * i] =
        Double2Short ( (double) ( pInData[iNumInChan * i + 3] * _MAXSHORT ) +
                       (double) pSound->vecsTmpAudioSndCrdStereo[2 * i] );
    pSound->vecsTmpAudioSndCrdStereo[2 * i + 1] =
        Double2Short ( (double) ( pInData[iNumInChan * i + 3] * _MAXSHORT ) +
                       (double) pSound->vecsTmpAudioSndCrdStereo[2 * i + 1] );
}
*/

            }
        }
        else if ( inInputData && inInputData->mNumberBuffers == (UInt32) iNumInChan && // we should have a matching number of buffers to channels
                  inInputData->mBuffers[0].mDataByteSize == static_cast<UInt32> ( iCoreAudioBufferSizeMono * 4 ) )
        {
            // one buffer per channel mode:
            AudioBuffer left       = inInputData->mBuffers[iSelInputLeftChannel];
            Float32*    pLeftData  = static_cast<Float32*> ( left.mData );
            AudioBuffer right      = inInputData->mBuffers[iSelInputRightChannel];
            Float32*    pRightData = static_cast<Float32*> ( right.mData );

            // copy input data
            for ( int i = 0; i < iCoreAudioBufferSizeMono; i++ )
            {
                // left
                pSound->vecsTmpAudioSndCrdStereo[2 * i] = (short) ( pLeftData[i] * _MAXSHORT );

                // right
                pSound->vecsTmpAudioSndCrdStereo[2 * i + 1] = (short) ( pRightData[i] * _MAXSHORT );
            }
        }
        else
        {
            // incompatible sizes, clear work buffer
            pSound->vecsTmpAudioSndCrdStereo.Reset ( 0 );
        }

        // call processing callback function
        pSound->ProcessCallback ( pSound->vecsTmpAudioSndCrdStereo );
    }

    if ( inDevice == pSound->CurrentAudioOutputDeviceID )
    {
        // check size (float32 has four bytes)
        if ( outOutputData && outOutputData->mBuffers[0].mDataByteSize ==
             static_cast<UInt32> ( iCoreAudioBufferSizeMono * iNumOutChan * 4 ) )
        {
            // one buffer with all the channels in interleaved format:
            // get a pointer to the input data of the correct type
            Float32* pOutData = static_cast<Float32*> ( outOutputData->mBuffers[0].mData );

            // copy output data
            for ( int i = 0; i < iCoreAudioBufferSizeMono; i++ )
            {
                // left
                pOutData[iNumOutChan * i + iSelOutputLeftChannel] =
                    (Float32) pSound->vecsTmpAudioSndCrdStereo[2 * i] / _MAXSHORT;

                // right
                pOutData[iNumOutChan * i + iSelOutputRightChannel] =
                    (Float32) pSound->vecsTmpAudioSndCrdStereo[2 * i + 1] / _MAXSHORT;
            }
        }
        else if ( outOutputData && outOutputData->mNumberBuffers == (UInt32) iNumOutChan && // we should have a matching number of buffers to channels
                  outOutputData->mBuffers[0].mDataByteSize == static_cast<UInt32> ( iCoreAudioBufferSizeMono * 4 ) )
        {
            // Outputs are to individual buffers too, rather than using channels
            Float32* pLeftOutData  = static_cast<Float32*> ( outOutputData->mBuffers[iSelOutputLeftChannel].mData );
            Float32* pRightOutData = static_cast<Float32*> ( outOutputData->mBuffers[iSelOutputRightChannel].mData );

            // copy output data
            for ( int i = 0; i < iCoreAudioBufferSizeMono; i++ )
            {
                // left
                pLeftOutData[i] = (Float32) pSound->vecsTmpAudioSndCrdStereo[2 * i] / _MAXSHORT;

                // right
                pRightOutData[i] = (Float32) pSound->vecsTmpAudioSndCrdStereo[2 * i + 1] / _MAXSHORT;
            }
        }
    }

    return kAudioHardwareNoError;
}

void CSound::callbackMIDI ( const MIDIPacketList* pktlist,
                            void*                 refCon,
                            void* )
{
    CSound* pSound = static_cast<CSound*> ( refCon );

    if ( pSound->midiInPortRef != static_cast<MIDIPortRef> ( NULL ) )
    {
        MIDIPacket* midiPacket = const_cast<MIDIPacket*> ( pktlist->packet );

        for ( unsigned int j = 0; j < pktlist->numPackets; j++ )
        {
            // copy packet and send it to the MIDI parser
            CVector<uint8_t> vMIDIPaketBytes ( midiPacket->length );
            for ( int i = 0; i < midiPacket->length; i++ )
            {
                vMIDIPaketBytes[i] = static_cast<uint8_t> ( midiPacket->data[i] );
            }
            pSound->ParseMIDIMessage ( vMIDIPaketBytes );

            midiPacket = MIDIPacketNext ( midiPacket );
        }
    }
}

bool CSound::ConvertCFStringToQString ( const CFStringRef stringRef,
                                        QString&          sOut )
{
    // check if the string reference is a valid pointer
    if ( stringRef != NULL )
    {
        // first check if the string is not empty
        if ( CFStringGetLength ( stringRef ) > 0 )
        {
            // convert CFString in c-string (quick hack!) and then in QString
            char* sC_strPropValue =
                (char*) malloc ( CFStringGetLength ( stringRef ) * 3 + 1 );

            if ( CFStringGetCString ( stringRef,
                                      sC_strPropValue,
                                      CFStringGetLength ( stringRef ) * 3 + 1,
                                      kCFStringEncodingUTF8 ) )
            {
                sOut = sC_strPropValue;
                free ( sC_strPropValue );

                return true; // OK
            }
        }

        // release the string reference because it is not needed anymore
        CFRelease ( stringRef );
    }

    return false; // not OK
}

OSStatus CSound::SetOutputDeviceAsCurrent(AudioDeviceID out)
{
    UInt32 size = sizeof(AudioDeviceID);;
    OSStatus err = noErr;
    
//        UInt32 propsize = sizeof(Float32);
    
    //AudioObjectPropertyScope theScope = mIsInput ? kAudioDevicePropertyScopeInput : kAudioDevicePropertyScopeOutput;
    
    AudioObjectPropertyAddress theAddress = { kAudioHardwarePropertyDefaultOutputDevice,
                                              kAudioObjectPropertyScopeGlobal,
                                              kAudioObjectPropertyElementMaster };
    
    if(out == kAudioDeviceUnknown) //Retrieve the default output device
    {
        err = AudioObjectGetPropertyData(kAudioObjectSystemObject, &theAddress, 0, NULL, &size, &out);
        checkErr(err);
    }
    mOutputDevice.Init(out, false);
    
    //Set the Current Device to the Default Output Unit.
    err = AudioUnitSetProperty(mOutputUnit,
                              kAudioOutputUnitProperty_CurrentDevice,
                              kAudioUnitScope_Global,
                              0,
                              &mOutputDevice.mID,
                              sizeof(mOutputDevice.mID));
                            
    return err;
}

OSStatus CSound::MakeGraph()
{
    OSStatus err = noErr;
    AudioComponentDescription varispeedDesc,outDesc;
    
    //Q:Why do we need a varispeed unit?
    //A:If the input device and the output device are running at different sample rates
    //we will need to move the data coming to the graph slower/faster to avoid a pitch change.
    varispeedDesc.componentType = kAudioUnitType_FormatConverter;
    varispeedDesc.componentSubType = kAudioUnitSubType_Varispeed;
    varispeedDesc.componentManufacturer = kAudioUnitManufacturer_Apple;
    varispeedDesc.componentFlags = 0;
    varispeedDesc.componentFlagsMask = 0;
  
    outDesc.componentType = kAudioUnitType_Output;
    outDesc.componentSubType = kAudioUnitSubType_DefaultOutput;
    outDesc.componentManufacturer = kAudioUnitManufacturer_Apple;
    outDesc.componentFlags = 0;
    outDesc.componentFlagsMask = 0;
    
    //////////////////////////
    ///MAKE NODES
    //This creates a node in the graph that is an AudioUnit, using
    //the supplied ComponentDescription to find and open that unit
    err = AUGraphAddNode(mGraph, &varispeedDesc, &mVarispeedNode);
    checkErr(err);
    err = AUGraphAddNode(mGraph, &outDesc, &mOutputNode);
    checkErr(err);
    
    //Get Audio Units from AUGraph node
    err = AUGraphNodeInfo(mGraph, mVarispeedNode, NULL, &mVarispeedUnit);
    checkErr(err);
    err = AUGraphNodeInfo(mGraph, mOutputNode, NULL, &mOutputUnit);
    checkErr(err);
    
    // don't connect nodes until the varispeed unit has input and output formats set

    return err;
}
