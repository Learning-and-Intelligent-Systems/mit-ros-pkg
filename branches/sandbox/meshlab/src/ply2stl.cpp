#include <ply2stl.h>
#include <io_base/baseio.h>


void ply2stl(const char* infile,const char* outfile) {
  BaseMeshIOPlugin b;
  MeshDocument *parent = new MeshDocument();
  MeshModel m(parent);
  RichParameterSet r;
  RichBool *rb = new RichBool(QString("Binary"),true,
			      true,QString(""),QString(""));
  r.addParam(rb);

  int mask=0;
  b.open(QString("ply"), QString(infile), m, mask, r);
  b.save(QString("stl"), QString(outfile), m, mask, r);
  //delete rb;
  delete parent;
}

void ply2wrl(const char* infile,const char* outfile) {
  BaseMeshIOPlugin b;
  MeshDocument *parent = new MeshDocument();
  MeshModel m(parent);
  RichParameterSet r;
  RichBool *rb = new RichBool(QString("Binary"),true,
               true,QString(""),QString(""));
  r.addParam(rb);

  int mask=0;
  b.open(QString("ply"), QString(infile), m, mask, r);
  b.save(QString("wrl"), QString(outfile), m, mask, r);
  //delete rb;
  delete parent;
}
