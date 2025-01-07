from gtts import gTTS

import os

mytext = 'Woi Hitam'
language = 'id'
myobj = gTTS(text=mytext, lang=language, slow=False)

myobj.save("welcome.mp3")
os.system("start welcome.mp3")