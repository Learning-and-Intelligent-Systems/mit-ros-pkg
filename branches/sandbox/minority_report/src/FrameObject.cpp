#include "FrameObject.h"




/** \brief  loads an image for the frame  */
void FrameObject::gtextureload(std::string filename){

//   glEnable(GL_DEPTH_TEST);
//   glShadeModel(GL_FLAT);
   // make ImageMagick object / read file into memory
//   std::cout<<filename<<std::endl;
   Magick::Image image(filename);
   // set RGBA output format
   image.magick("RGBA");
   // write to BLOB in RGBA format
   image.write(&blob);
   imgcols=image.columns();
   imgrows=image.rows();
   textureloaded=true;
}

/** \brief  Draw image for frame object  */
void FrameObject::DrawTexture(){
   //load the last part of the texture while in the opengl frame...
   if(!fullyloaded){
      // create the temporary Texture
      glGenTextures(1, &texture);
      // bind the texture to the next thing we make
      glBindTexture(GL_TEXTURE_2D, texture);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
      gluBuild2DMipmaps(GL_TEXTURE_2D, 4, imgcols, imgrows, GL_RGBA, GL_UNSIGNED_BYTE, blob.data());
      fullyloaded=true;
   }

   float tpoints[4][2]={{0.0f,0.0f},{0.0f,1.0f},{1.0f,1.0f},{1.0f,0.0f}};
   float ppoints[4][2]={{-0.5f,0.5f},{-0.5f,-0.5f},{0.5f,-0.5f},{0.5f,0.5f}};

//   glDisable(GL_DEPTH_TEST);
   glEnable(GL_TEXTURE_2D);
   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
   glBindTexture( GL_TEXTURE_2D, texture );
   glBegin(GL_QUADS);
   for(int i=0;i<4;i++){
      glTexCoord2d(tpoints[i][0],tpoints[i][1]);
      glVertex3f(width*(ppoints[i][0]),height*(ppoints[i][1]),0.0);
   }
   glEnd();
   glFlush();
//   glEnable(GL_DEPTH_TEST);
   glDisable(GL_TEXTURE_2D);

}

/** \brief  Draw blank frame  */
void FrameObject::DrawSolid(){
   glDisable(GL_TEXTURE_2D);
   glColor3f(0.0f,0.0f,1.0f);
   if(highlighted)
      glColor4f(1.0f,1.0f,0.0f,.1f);
   if(framed)
      glColor4f(0.0f,1.0f,0.0f,.1f);

   glBegin(GL_QUADS);
   glVertex3f(-width/2.0,height/2.0,0.0f);
   glVertex3f(width/2.0,height/2.0,0.0f);
   glVertex3f(width/2.0,-height/2.0,0.0f);
   glVertex3f(-width/2.0,-height/2.0,0.0f);
   glEnd();
   glEnable(GL_TEXTURE_2D);

}




/** \brief  Draw an outline around the frame  */
void FrameObject::DrawOutline(){
   glDisable(GL_TEXTURE_2D);
   float rstart=0.0, gstart=0.0, bstart=0.0, astart=0.5;
   float rend=0.0,   gend=0.0,   bend=0.0,   aend=0.5;
   float osize=.01,border=0.002;
//   glColor4f(1.0f,1.0f,1.0f,.9f);
//   if(highlighted){
//	   rstart=1.0; gstart=0.0; bstart=0.0;
//   }
//   if(framed){
//	   rstart=0.0; gstart=1.0; bstart=0.0;
//
//
//   }
   gstart=std::max(frameintensity, highlightedintensity);
   rstart= highlightedintensity;



   //fading left side
   glBegin(GL_QUADS);
   glColor4f(rend,gend,bend,aend);
   glVertex3f(-width/2.0-osize-border, height/2.0+osize+border,-.0001f);
   glColor4f(rstart,gstart,bstart,astart);
   glVertex3f(-width/2.0-border,	   height/2.0+border,-.0001f);
   glVertex3f(-width/2.0-border,  	  -height/2.0-border,-.0001f);
   glColor4f(rend,gend,bend,aend);
   glVertex3f(-width/2.0-osize-border,-height/2.0-osize-border,-.0001f);
   glEnd();

   //fading top
   glBegin(GL_QUADS);
   glColor4f(rend,gend,bend,aend);
   glVertex3f(-width/2.0-osize-border, height/2.0+osize+border,-.0001f);
   glVertex3f(width/2.0+osize+border,  height/2.0+osize+border,-.0001f);
   glColor4f(rstart,gstart,bstart,astart);
   glVertex3f(width/2.0+border,  	  height/2.0+border,-.0001f);
   glVertex3f(-width/2.0-border,	   height/2.0+border,-.0001f);
   glEnd();


   //fading right side
   glBegin(GL_QUADS);
   glColor4f(rend,gend,bend,aend);
   glVertex3f(-1.0*(-width/2.0-osize-border), -1.0*(height/2.0+osize+border),-.0001f);
   glColor4f(rstart,gstart,bstart,astart);
   glVertex3f(-1.0*(-width/2.0-border),	   -1.0*(height/2.0+border),-.0001f);
   glVertex3f(-1.0*(-width/2.0-border),  	 -1.0*( -height/2.0-border),-.0001f);
   glColor4f(rend,gend,bend,aend);
   glVertex3f(-1.0*(-width/2.0-osize-border),-1.0*(-height/2.0-osize-border),-.0001f);
   glEnd();

   //fading bottom
   glBegin(GL_QUADS);
   glColor4f(rend,gend,bend,aend);
   glVertex3f(width/2.0+osize+border, -1.0*(height/2.0+osize+border),-.0001f);
   glVertex3f(-width/2.0-osize-border, -1.0*( height/2.0+osize+border),-.0001f);
   glColor4f(rstart,gstart,bstart,astart);
   glVertex3f(-1.0*(width/2.0+border),  	 -1.0*( height/2.0+border),-.0001f);
   glVertex3f(-1.0*(-width/2.0-border),	  -1.0*( height/2.0+border),-.0001f);
   glEnd();



//   glBegin(GL_QUADS);
//   glVertex3f(-width/2.0-.001,height/2.0+.001,-.0001f);
//   glVertex3f(width/2.0+.001,height/2.0+.001,-.0001f);
//   glVertex3f(width/2.0+.001,-height/2.0-.001,-.0001f);
//   glVertex3f(-width/2.0-.001,-height/2.0-.001,-.0001f);
//   glEnd();
//
   glEnable(GL_TEXTURE_2D);

}

/** \brief  Main Draw function  */
void FrameObject::Draw(){
   if(sweeping){
      pose.orientation=pose.orientation*sweeprot;
      pose.position.x-=.01;
   }
   updateintensities();
	glPushMatrix();

    glDisable(GL_DEPTH_TEST);
	   glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   double _tcam[16];
   pose.toOpenGLMatrix(_tcam);
   glMultMatrixd(_tcam);
   if(textureloaded)
      DrawTexture();
   else
      DrawSolid();



   	   DrawOutline();

      glEnable(GL_DEPTH_TEST);

	glPopMatrix();
}


/** \brief there has been a sweep action. If this frame is within rad pixels of the sweep, send it into sweep mode */
   void FrameObject::sweepCheck(double winy, double rad){
//      std::cout<<"sweep check "<<getVPCoords(pose.position).y<<"  @  "<<winy<<"  rad:  "<<rad<<std::endl;
      if(fabs(getVPCoords(pose.position).y-winy)<rad){
         sweeping=true;
//         sweeprot.generate(); //generate a random quaternion to rotate with
         sweeprot.fromEuler(0.0,(((double)(rand()%100))/500.0-.1),0.0);
      }

   }

   /** \brief indicate that the give pose (in window coords) is in a quadrant diagonal from frame
   /return the quadrant number -> 1: top right,  2: top left, 3: bottom left, 4 bottom right
   */
   int FrameObject::isInCorner(gatmo_point3D_t p){
      if(sweeping) return 0;
      //TODO: add a transform so this works even if the frame is rotated

      gatmo_point3D_t temp(width/2.0,height/2.0,0.0);
      gatmo_point3D_t upperwin = getVPCoords(pose.position+temp);
      gatmo_point3D_t lowerwin = getVPCoords(pose.position-temp);
      if(p.x > upperwin.x){
         if(p.y < lowerwin.y)
            return 4;
         if(p.y > upperwin.y)
            return 1;
      }
      if(p.x < lowerwin.x){
         if(p.y < lowerwin.y)
            return 3;
         if(p.y > upperwin.y)
            return 2;
      }
      return 0;
   }

   //indicate that the give line (in window coords) goes close to this object
   bool FrameObject::isCloseLine(gatmo_line3d_t &l){
      if(sweeping) return false;
      //TODO: add a transform so this works even if the frame is rotated
      gatmo_point3D_t temp(width/2.0,height/2.0,0.0);
      gatmo_point3D_t vppt = getVPCoords(pose.position);
      vppt.z=0;
      gatmo_point3D_t upperwin = getVPCoords(pose.position+temp);
      gatmo_point3D_t lowerwin = getVPCoords(pose.position-temp);
      if(l.distto(vppt) < lowerwin.dist(upperwin)/2.0)
         return true;
      return false;
   }


   bool FrameObject::isOn(gatmo_pose6D_t p){
      if(sweeping) return false;
      gatmo_point3D_t temp(width/2.0,height/2.0,0.0);
      gatmo_point3D_t upperwin = getVPCoords(pose.position+temp);
      gatmo_point3D_t lowerwin = getVPCoords(pose.position-temp);
      gatmo_point3D_t hwin = getVPCoords(p.position);
//      printf("FrameObject::isOn: is %.03f > %.03f > %.03f  and %.03f > %.03f > %.03f",
//            upperwin.x,hwin.x,lowerwin.x,upperwin.y,hwin.y,lowerwin.y);
//      std::cout<<std::endl;
      if(hwin.x > lowerwin.x && hwin.x < upperwin.x && hwin.y > lowerwin.y && hwin.y < upperwin.y){
         highlighted=true;
         return true;
      }
      return false;
   }

   void FrameObject::update(gatmo_point3D_t pold, gatmo_point3D_t pnew){
      if(sweeping) return;
      pose.position=Updatepose(pose.position,pold,pnew);
   }





   gatmo_point3D_t getVPCoords(gatmo_point3D_t pt){
      gatmo_point3D_t out;
      glPushMatrix();
   // GG_MAIN::gatmoui.loadGlobalFrame();
         glLoadIdentity();
         glRotatef(-1.5707,00,0.0,1.0);
         glRotatef(-1.5707,00,1.0,0.0);
         glTranslatef(0,0,-1.50);
      GLint viewport[4];
      GLdouble modelview[16];
      GLdouble projection[16];

      glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
      glGetDoublev( GL_PROJECTION_MATRIX, projection );
      glGetIntegerv( GL_VIEWPORT, viewport );

      GLdouble junkX, junkY, junkz;
      gluProject( pt.x, pt.y, pt.z, modelview, projection, viewport, &junkX, &junkY, &junkz);
      out.x=junkX; out.y= junkY; out.z=junkz;
      glPopMatrix();
      return out;
   }


   //find the location an object should go to to be at the specified window coordinates
   gatmo_point3D_t getWorldCoords(gatmo_point3D_t pt){
      gatmo_point3D_t out;
      glPushMatrix();
   // GG_MAIN::gatmoui.loadGlobalFrame();
         glLoadIdentity();
      //   glRotatef(-1.5707,00,0.0,1.0);
      //   glRotatef(-1.5707,00,1.0,0.0);
         glTranslatef(0,0,-1.50);
      GLint viewport[4];
      GLdouble modelview[16];
      GLdouble projection[16];

      glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
      glGetDoublev( GL_PROJECTION_MATRIX, projection );
      glGetIntegerv( GL_VIEWPORT, viewport );

      GLdouble junkX, junkY, junkz;
      gluUnProject( pt.x, pt.y, pt.z, modelview, projection, viewport, &junkX, &junkY, &junkz);
      out.x=junkX; out.y= junkY; out.z=junkz;
      glPopMatrix();
      return out;
   }

   //update obj in world coordinates so it moves as much as the difference between oldref and newref
   //in screen coordinates
   gatmo_point3D_t Updatepose(gatmo_point3D_t obj, gatmo_point3D_t oldref, gatmo_point3D_t newref){
      gatmo_point3D_t out;
        glPushMatrix();
        glLoadIdentity();
        glTranslatef(0,0,-1.50);
        GLint viewport[4];
        GLdouble modelview[16];
        GLdouble projection[16];

        glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
        glGetDoublev( GL_PROJECTION_MATRIX, projection );
        glGetIntegerv( GL_VIEWPORT, viewport );

        GLdouble oldwinx,oldwiny,oldwinz, newwinx,newwiny,newwinz, objwinx,objwiny,objwinz;
        GLdouble newx,newy,newz;
        //get window coords for obj, oldref and new ref
        gluProject( obj.x, obj.y, obj.z, modelview, projection, viewport, &objwinx, &objwiny, &objwinz);
        gluProject( oldref.x, oldref.y, oldref.z, modelview, projection, viewport, &oldwinx, &oldwiny, &oldwinz);
        gluProject( newref.x, newref.y, newref.z, modelview, projection, viewport, &newwinx, &newwiny, &newwinz);
        objwinx+=(newwinx-oldwinx);
        objwiny+=(newwiny-oldwiny);
        //project the new object window coordinates back into world coordinates
        gluUnProject( objwinx, objwiny, objwinz, modelview, projection, viewport, &newx, &newy, &newz);
        out.x=newx,out.y=newy;out.z=newz;
        return out;
   }


