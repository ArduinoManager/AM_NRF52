#include "FileManager.h"

FileManager::FileManager() {
	InternalFS.begin(); // Initialize Internal File System
}

bool FileManager::append(String & fileName, uint8_t *byte, size_t size) {
	File f(InternalFS);
	
  f.open(fileName.c_str(), FILE_O_WRITE);

  if (f) {

    uint8_t ret = f.write(byte, size);
    f.close();

    return (ret == size);
  }
  return false;
}

void FileManager::deleteFile(String &fileName) {
  InternalFS.remove(fileName.c_str());
}

bool FileManager::read(String &fileName, uint8_t position, uint8_t *byte, size_t size) {
	File f(InternalFS);
	
  f.open(fileName.c_str(), FILE_O_READ);

  if (f) {
    bool ok;

    ok = f.seek(position * size);
    if (ok)
      ok = f.read(byte, size);

    f.close();

    return ok;
  }

  return false;
}

bool FileManager::update(String &fileName, uint8_t position, uint8_t *byte, size_t size) {
  File f(InternalFS);
  
  f.open(fileName.c_str(), FILE_O_WRITE);

  if (f) {
    bool ok;
    
    ok = f.seek(position * size);
    if (ok)
      ok = f.write(byte, size);

    f.close();
    return ok;
  }

  return false;
}

bool FileManager::copy(String &sourcefileName, const char *destinationfileName) {

  String d = String(destinationfileName);
  return copy(sourcefileName, d);
}

bool FileManager::copy(const char *sourcefileName, String &destinationfileName) {

  String d = String(sourcefileName);
  return copy(d, destinationfileName);
}

bool FileManager::copy(String &sourcefileName, String &destinationfileName) {
  uint8_t buffer[64];
  File sourceFile(InternalFS);
  File destinationFile(InternalFS);
  
  if (InternalFS.exists(destinationfileName.c_str())) {  
    bool ret = InternalFS.remove(destinationfileName.c_str());
    if (!ret)
      return false;
  }
  
	sourceFile.open(sourcefileName.c_str(), FILE_O_READ);
  if (sourceFile) {
    destinationFile.open(destinationfileName.c_str(), FILE_O_WRITE);
    if (!destinationFile) {
      sourceFile.close();
      return false;
    }

    uint8_t ret = sourceFile.read(buffer, 64);

    while (ret > 0) {
      destinationFile.write(buffer, ret);
      ret = sourceFile.read(buffer, 64);
    }
    
    destinationFile.close();
    sourceFile.close();
    return true;
  }

  return false;
}

bool FileManager::remove(String &fileName, uint8_t position, size_t size) {
	File sourceFile(InternalFS);
	File tmpFile(InternalFS);
	
  sourceFile.open(fileName.c_str(), FILE_O_READ);
  tmpFile.open("tmp.txt", FILE_O_WRITE);
  
  if (sourceFile) {  
    if (!tmpFile) {
      sourceFile.close();
      return false;
    }

    uint32_t length = sourceFile.size();
    uint8_t  buffer[size];

    for (int i = 0; i < length / size; i++) {

      if (i != position) {

        bool ok = sourceFile.seek((uint32_t)i * size);
        if (!ok) {
          sourceFile.close();
          tmpFile.close();
          return false;
        }

        ok = sourceFile.read(buffer, size);
        if (!ok) {
          sourceFile.close();
          tmpFile.close();
          return false;
        }

        ok = tmpFile.write(buffer, size);
        if (!ok) {
          sourceFile.close();
          tmpFile.close();
          return false;
        }
      }
    }

    sourceFile.close();
    tmpFile.flush();
    tmpFile.close();

    deleteFile(fileName);

    if (!copy("tmp.txt", fileName))
      return false;

    InternalFS.remove("tmp.txt");
    return true;
  }

  return false;
}

int FileManager::find(String &fileName, uint8_t *byte, size_t size, bool (*check)(uint8_t *pRecord, void *pData), void *pData) {
	File f(InternalFS);
  
  f.open(fileName.c_str(), FILE_O_READ);
  if (f) {

    bool ok;
    uint16_t length = f.size();

    for (int i = 0; i < length / size; i++) {

      ok = f.seek(i * size);
      if (!ok) {

        f.close();
        return -1;
      }

      ok = f.read(byte, size);
      if (!ok) {
        f.close();
        return -1;
      }

      ok = (*check)(byte,pData);

      if (ok) {
        f.close();
        return i;
      }
    }

    f.close();
  }
  
  return -1;
}

