#include "GatmoPluginDisplay.h"

GatmoPluginDisplay::GatmoPluginDisplay(wxWindow * parent):GatmoDisplayInterface(parent){

	//GatmoDisplayObject part:
	//should always do this
	sprintf(name,"Sample plugin");
	display=false;
	emph=false;


	//setup gui display
	InitGui();

	//add to the global display list
	AddtoDisplayList();

}

GatmoPluginDisplay::~GatmoPluginDisplay()
{

}

void GatmoPluginDisplay::Draw(){
	static float currentrot=0.0;
	currentrot+=3;
	wxString str=numres->GetValue();
	long l;
	str.ToLong((long*)&l);
	int numclicks=l;

	glPushMatrix();
	for(int i=0; i<numclicks; i++){
		glTranslatef(.40,0,0);
//		glRotatef(currentrot,1,1,1);
		glColor4f((float)i/(float)numclicks,0.0,(float)(numclicks-i)/numclicks,0.0);
		glutWireTeapot(.10);
		glRotatef(-currentrot,1,currentrot,1);
	}
	glPopMatrix();

}
void GatmoPluginDisplay::DrawAlt(){
	//you can put a different draw function here - this function just give you the option
	//of choosing between draw functions without knowing anything about the class
	Draw();

}





///this function initializes the gui panel
void GatmoPluginDisplay::InitGui(){

	wxBoxSizer* v_sizer = new wxBoxSizer(wxVERTICAL);
	//add title
	wxStaticText *lbltitle = new wxStaticText(this, wxID_ANY, wxT("Map Control"));
	v_sizer->Add(lbltitle,1,wxEXPAND | wxALIGN_CENTER | wxALL, 5 );

	//add button
	wxButton* mybutton = new wxButton(this, wxID_ANY, wxT("Click Me!"));
	//connect the button to a function that gets called when it is pressed
    Connect(mybutton->GetId(), wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(GatmoPluginDisplay::OnButtonPress));

	//add checkbox to say whether to display stuff
    drawenablecb = new wxCheckBox(this, -1, wxT("Draw Stuff"), wxPoint(20, 20));
    Connect(drawenablecb->GetId(), wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(GatmoPluginDisplay::DrawEnableCB));

	//add textcontrol to show number of clicks
	wxStaticText *lblres = new wxStaticText(this, wxID_ANY, wxT("Number of clicks"));
	numres = new wxTextCtrl(this, wxID_ANY, wxT("0"),wxDefaultPosition, wxSize(25,30));
	wxBoxSizer* v_sizer_res = new wxBoxSizer(wxVERTICAL);
	v_sizer_res->Add(lblres,1,wxEXPAND | wxALIGN_CENTER);
	v_sizer_res->Add(numres,1,wxEXPAND | wxALIGN_CENTER);


	//chuck it all in a sizer
	wxBoxSizer* h_sizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* v_sizer_load = new wxBoxSizer(wxVERTICAL);
	v_sizer_load->Add(mybutton,1,wxEXPAND | wxALIGN_CENTER);
	v_sizer_load->Add(drawenablecb,1);
	h_sizer->Add(v_sizer_load,1,wxEXPAND | wxALIGN_CENTER );
	h_sizer->Add(v_sizer_res,1,wxEXPAND | wxALIGN_CENTER | wxLEFT, 5 );
	v_sizer->Add(h_sizer,0);

	this->SetSizer(v_sizer);
    this->Show();

    drawenablecb->Disable();  //don't enable drawing  unless button has been clicked
}

//This function is a bit roundabout, but it demonstrates
//a safe way to get a value from and write a value to a textcontrol
void GatmoPluginDisplay::OnButtonPress(wxCommandEvent& event){
	//get value of counter display:
	wxString str=numres->GetValue();
	long l;
	str.ToLong((long*)&l);
	int numclicks=l;

	//Increment value:
	numclicks++;

	//write value back to control:
	str.Clear();
	str<<numclicks;
	numres->SetValue(str);
	drawenablecb->Enable();
}



void GatmoPluginDisplay::DrawEnableCB(wxCommandEvent& event){
	display=false;
	if(event.IsChecked())
		display=true;  //this will tell the OpenGL draw loop to call our Draw function.

}









