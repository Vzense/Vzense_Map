#ifndef VZENSEMAP_API
#define VZENSEMAP_API

#ifdef PS_EXPORT_ON
    #ifdef _WIN32
        #define VZENSE_API_EXPORT __declspec(dllexport)
    #else
        #define VZENSE_API_EXPORT __attribute__((visibility("default")))
    #endif
#else
    #ifdef _WIN32
        #define VZENSE_API_EXPORT __declspec(dllimport)
    #else
        #define VZENSE_API_EXPORT __attribute__((visibility("default")))
    #endif
#endif

#ifdef __cplusplus
#define VZENSE_C_API_EXPORT extern "C" VZENSE_API_EXPORT
#else
#define VZENSE_C_API_EXPORT VZENSE_API_EXPORT
#endif

typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
 
enum Vz_ElemType { ElemNone, U8C1, U8C3, U16C1, ElemMAX };

#pragma pack (push, 1)
struct Vz_MapParams
{
	double	tofIntrinsic_fx;	//!< tof Intrinsic Focal length x (pixel)
	double	tofIntrinsic_fy;	//!< tof Intrinsic Focal length y (pixel)
	double	tofIntrinsic_cx;	//!< tof Intrinsic Principal point x (pixel)
	double	tofIntrinsic_cy;	//!< tof Intrinsic Principal point y (pixel)

	double	rgbIntrinsic_fx;	//!< rgb Intrinsic Focal length x (pixel)
	double	rgbIntrinsic_fy;	//!< rgb Intrinsic Focal length y (pixel)
	double	rgbIntrinsic_cx;	//!< rgb Intrinsic Principal point x (pixel)
	double	rgbIntrinsic_cy;	//!< rgb Intrinsic Principal point y (pixel)


	double rotation[9];			//!< Orientation stored as an array of 9 double representing a 3x3 rotation matrix.
	double transfer[3];			//!< Location stored as an array of 3 double representing a 3-D translation vector.

	uint16_t tofFrameWidth;		//!< tof frame width
	uint16_t tofFrameHeigth;	//!< tof frame height
	uint16_t rgbFrameWidth;		//!< rgb frame width
	uint16_t rgbFrameHeigth;	//!< rgb frame height
};

struct Vz_Frame 
{
	Vz_ElemType type;
	unsigned int w;
	unsigned int h;
	unsigned char* pBuf;
	Vz_Frame() :type(ElemNone),w(0),h(0), pBuf(0)
	{}
	void ShallowCopyTo(Vz_Frame& psFrame)
	{
		psFrame.type = type;
		psFrame.w = w;
		psFrame.h = h;
		psFrame.pBuf = pBuf;
	}
};

#pragma pack (pop)


class APIMapProcess
{

public:

	virtual ~APIMapProcess() {};
	/*!
	*  Init
	*  @Parameters:
	*      const Vz_MapParams& params[In]: internal parameters, external parameters, distortion parameters of tof and color lens
    *      const char* calibFilePath:   the  Calib File Path for camera which index is deviceIndex.
	*  @Return: 0: OK
	*/
	virtual int PreProcInit(const Vz_MapParams&  params)=0;

	/*!
	*  Shutdown
	*  @Parameters:void
	*  @Return: 0: OK
	*/
	virtual int PreProcShutDown(void)=0;


	/*!
	*  Depth frame with 16bits per pixel in mm that is mapped to RGB camera space and resolution is same as RGB frame's,
	*  @Parameters:
	*      Vz_Frame& rgb[In]:input rgb data;
	*      Vz_Frame& depth[In]:input depth data;
	*      Vz_Frame* out_depth[Out]:out mapped depth data;
	*  @Return: 0: OK
	*/
	virtual int PreProcRGB2Depth(Vz_Frame& rgb, Vz_Frame& depth, Vz_Frame* out_depth)=0;

	/*!
	*  RGB frame with 24bits that is mapped to depth camera space and resolution is same as depth frame's,
	*  @Parameters:
	*      const Vz_Frame& depth[In]:input depth data;
	*      const Vz_Frame& rgb[In]:input rgb data;
	*      Vz_Frame* out_rgb[Out]:out mapped rgb data;
	*  @Return: 0: OK
	*/
	virtual int PreProcDepth2RGB(Vz_Frame& depth,Vz_Frame& rgb, Vz_Frame* out_rgb)=0;

	/*!
	*  IR frame with 16bits per pixel that is mapped to RGB camera space and resolution is same as RGB frame's,
	*  @Parameters:
	*      Vz_Frame& rgb[In]:input rgb data;
	*      Vz_Frame& depth[In]:input depth data;
	*      Vz_Frame& ir[In]:input ir data;
	*      Vz_Frame* out_ir[Out]:out mapped ir data;
	*  @Return: 0: OK
	*/
	virtual int PreProcRGB2IR(Vz_Frame& rgb, Vz_Frame& depth, Vz_Frame& ir, Vz_Frame* out_ir) = 0;

	/*!
	*  IR frame and Depth frame with 16bits per pixel that is mapped to RGB camera space and resolution is same as RGB frame's,
	*  @Parameters:
	*      Vz_Frame& rgb[In]:input rgb data;
	*      Vz_Frame& depth[In]:input depth data;
	*      Vz_Frame& ir[In]:input ir data;
	*      Vz_Frame* out_depth[Out]:out mapped depth data;
	*      Vz_Frame* out_ir[Out]:out mapped ir data;
	*  @Return: 0: OK
	*/
	virtual int PreProcRGB2IRandDepth(Vz_Frame& rgb, Vz_Frame& depth, Vz_Frame& ir, Vz_Frame* out_depth, Vz_Frame* out_ir) = 0;
	/*!
	*  Resize the img resolution
	*  @Parameters:
	*   Vz_Frame& img[In][Out]:input and output img data
	*	destWidth[in]:		dest resolution width
	*	destHeigh[in]:		dest resolution height
	*   intertype[in]:      interpolation type  0: nearest interpolation   not 0:bilinear interpolation
	*  @Return:				0: OK
	*/

	virtual int PreProcImgResize(Vz_Frame& img, int destWidth, int destHeigh, int intertype = 1) = 0;

	int mSensorID;

};


VZENSE_C_API_EXPORT APIMapProcess * Get_APIMapProcess();

#endif // VZENSEMAP_API

