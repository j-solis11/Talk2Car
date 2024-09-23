import React, { useEffect, useState } from 'react';
import SpeechRecognition, { useSpeechRecognition } from 'react-speech-recognition';


const Dictaphone1 = () => {
 const [message, setMessage] = useState('');
 const commands = [
   {
     command: 'slow',
     callback: () => setMessage('0')
   },
   {
     command: 'fast',
     callback: () => setMessage('1')
   },
   {
     command: 'stop',
     callback: () => setMessage('2')
   },
   {
     command: 'left',
     callback: () => setMessage('3')
   },
   {
     command: 'right',
     callback: () => setMessage('4')
   },
   {
    command: 'straight',
    callback: () => setMessage('5')
  },
   {
    command: 'i like men',
    callback: () => setMessage('hi sourav')
  },
 ]
 const {
   transcript,
   interimTranscript,
   finalTranscript,
   resetTranscript,
   listening,
 } = useSpeechRecognition({ commands });


 useEffect(() => {
   if (finalTranscript !== '') {
     console.log('Got final result:', finalTranscript);
   }
 }, [interimTranscript, finalTranscript]);

useEffect(() => {
  if (message !== '') {
    sendDataToServer(message);
    console.log("Sent message to server")
  }
}, [message]);

function sendDataToServer(message) {
  fetch('http://localhost:8000/data', {
          method: 'POST',
          headers: {
              'Content-Type': 'application/json',
          },
          body: JSON.stringify({ message }),
  })
  .then(response => response.text())
  .then(message => console.log(message))
  .catch((error) => {
    console.error('Error: ', error);
  });
}

 if (!SpeechRecognition.browserSupportsSpeechRecognition()) {
   return null;
 }

 if (!SpeechRecognition.browserSupportsSpeechRecognition()) {
   console.log('Your browser does not support speech recognition software! Try Chrome desktop, maybe?');
 }
 const listenContinuously = () => {
   SpeechRecognition.startListening({
     continuous: true,
     language: 'en-GB',
   });
 };

 return (
   <div>
     <div>
       <span>
         listening:
         {' '}
         {listening ? 'on' : 'off'}
       </span>
       <div>
         <button type="button" onClick={resetTranscript}>Reset</button>
         <button type="button" onClick={listenContinuously}>Listen</button>
         <button type="button" onClick={SpeechRecognition.stopListening}>Stop</button>
       </div>
     </div>
     <div>
       {message}
     </div>
     <div>
       <span>{transcript}</span>
     </div>
   </div>
 );
};

export default Dictaphone1;
