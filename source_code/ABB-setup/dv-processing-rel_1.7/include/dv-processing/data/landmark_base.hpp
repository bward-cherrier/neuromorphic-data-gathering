// automatically generated by the FlatBuffers compiler, do not modify

#ifndef FLATBUFFERS_GENERATED_LANDMARK_DV_H_
#define FLATBUFFERS_GENERATED_LANDMARK_DV_H_

#include "../external/flatbuffers/flatbuffers.h"

#include "cstring.hpp"
#include "cvector.hpp"
#include "geometry_types_base.hpp"

namespace dv {

struct ObservationFlatbuffer;
struct Observation;

struct LandmarkFlatbuffer;
struct Landmark;

struct LandmarksPacketFlatbuffer;
struct LandmarksPacket;

bool operator==(const Observation &lhs, const Observation &rhs);
bool operator==(const Landmark &lhs, const Landmark &rhs);
bool operator==(const LandmarksPacket &lhs, const LandmarksPacket &rhs);

inline const flatbuffers::TypeTable *ObservationTypeTable();

inline const flatbuffers::TypeTable *LandmarkTypeTable();

inline const flatbuffers::TypeTable *LandmarksPacketTypeTable();

struct Observation : public flatbuffers::NativeTable {
	typedef ObservationFlatbuffer TableType;

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.Observation";
	}

	int32_t trackId;
	int32_t cameraId;
	dv::cstring cameraName;
	int64_t timestamp;

	Observation() : trackId(0), cameraId(0), timestamp(0) {
	}

	// Generated Constructor
	Observation(int32_t _trackId, int32_t _cameraId, const dv::cstring &_cameraName, int64_t _timestamp) :
		trackId{_trackId},
		cameraId{_cameraId},
		cameraName{_cameraName},
		timestamp{_timestamp} {
	}
};

inline bool operator==(const Observation &lhs, const Observation &rhs) {
	return (lhs.trackId == rhs.trackId) && (lhs.cameraId == rhs.cameraId) && (lhs.cameraName == rhs.cameraName)
		&& (lhs.timestamp == rhs.timestamp);
}

struct ObservationFlatbuffer FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
	typedef Observation NativeTableType;

	static const flatbuffers::TypeTable *MiniReflectTypeTable() {
		return ObservationTypeTable();
	}

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.Observation";
	}

	enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
		VT_TRACKID    = 4,
		VT_CAMERAID   = 6,
		VT_CAMERANAME = 8,
		VT_TIMESTAMP  = 10
	};

	/// The tracking sequence ID that the landmark is observed by a camera
	int32_t trackId() const {
		return GetField<int32_t>(VT_TRACKID, 0);
	}

	/// Arbitrary ID of the camera, this can be application specific
	int32_t cameraId() const {
		return GetField<int32_t>(VT_CAMERAID, 0);
	}

	/// Name of the camera. Optional.
	const flatbuffers::String *cameraName() const {
		return GetPointer<const flatbuffers::String *>(VT_CAMERANAME);
	}

	/// Timestamp of the observation (µs).
	int64_t timestamp() const {
		return GetField<int64_t>(VT_TIMESTAMP, 0);
	}

	bool Verify(flatbuffers::Verifier &verifier) const {
		return VerifyTableStart(verifier) && VerifyField<int32_t>(verifier, VT_TRACKID)
			&& VerifyField<int32_t>(verifier, VT_CAMERAID) && VerifyOffset(verifier, VT_CAMERANAME)
			&& verifier.VerifyString(cameraName()) && VerifyField<int64_t>(verifier, VT_TIMESTAMP)
			&& verifier.EndTable();
	}

	Observation *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	void UnPackTo(Observation *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	static void UnPackToFrom(
		Observation *_o, const ObservationFlatbuffer *_fb, const flatbuffers::resolver_function_t *_resolver = nullptr);
	static flatbuffers::Offset<ObservationFlatbuffer> Pack(flatbuffers::FlatBufferBuilder &_fbb, const Observation *_o,
		const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct ObservationBuilder {
	flatbuffers::FlatBufferBuilder &fbb_;
	flatbuffers::uoffset_t start_;

	void add_trackId(int32_t trackId) {
		fbb_.AddElement<int32_t>(ObservationFlatbuffer::VT_TRACKID, trackId, 0);
	}

	void add_cameraId(int32_t cameraId) {
		fbb_.AddElement<int32_t>(ObservationFlatbuffer::VT_CAMERAID, cameraId, 0);
	}

	void add_cameraName(flatbuffers::Offset<flatbuffers::String> cameraName) {
		fbb_.AddOffset(ObservationFlatbuffer::VT_CAMERANAME, cameraName);
	}

	void add_timestamp(int64_t timestamp) {
		fbb_.AddElement<int64_t>(ObservationFlatbuffer::VT_TIMESTAMP, timestamp, 0);
	}

	explicit ObservationBuilder(flatbuffers::FlatBufferBuilder &_fbb) : fbb_(_fbb) {
		start_ = fbb_.StartTable();
	}

	ObservationBuilder &operator=(const ObservationBuilder &);

	flatbuffers::Offset<ObservationFlatbuffer> Finish() {
		const auto end = fbb_.EndTable(start_);
		auto o         = flatbuffers::Offset<ObservationFlatbuffer>(end);
		return o;
	}
};

inline flatbuffers::Offset<ObservationFlatbuffer> CreateObservation(flatbuffers::FlatBufferBuilder &_fbb,
	int32_t trackId = 0, int32_t cameraId = 0, flatbuffers::Offset<flatbuffers::String> cameraName = 0,
	int64_t timestamp = 0) {
	ObservationBuilder builder_(_fbb);
	builder_.add_timestamp(timestamp);
	builder_.add_cameraName(cameraName);
	builder_.add_cameraId(cameraId);
	builder_.add_trackId(trackId);
	return builder_.Finish();
}

inline flatbuffers::Offset<ObservationFlatbuffer> CreateObservationDirect(flatbuffers::FlatBufferBuilder &_fbb,
	int32_t trackId = 0, int32_t cameraId = 0, const char *cameraName = nullptr, int64_t timestamp = 0) {
	auto cameraName__ = cameraName ? _fbb.CreateString(cameraName) : 0;
	return dv::CreateObservation(_fbb, trackId, cameraId, cameraName__, timestamp);
}

flatbuffers::Offset<ObservationFlatbuffer> CreateObservation(flatbuffers::FlatBufferBuilder &_fbb,
	const Observation *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

struct Landmark : public flatbuffers::NativeTable {
	typedef LandmarkFlatbuffer TableType;

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.Landmark";
	}

	Point3f pt;
	int64_t id;
	int64_t timestamp;
	dv::cvector<int8_t> descriptor;
	dv::cstring descriptorType;
	dv::cvector<float> covariance;
	dv::cvector<Observation> observations;

	Landmark() : id(0), timestamp(0) {
	}

	// Generated Constructor
	Landmark(const Point3f &_pt, int64_t _id, int64_t _timestamp, const dv::cvector<int8_t> &_descriptor,
		const dv::cstring &_descriptorType, const dv::cvector<float> &_covariance,
		const dv::cvector<Observation> &_observations) :
		pt{_pt},
		id{_id},
		timestamp{_timestamp},
		descriptor{_descriptor},
		descriptorType{_descriptorType},
		covariance{_covariance},
		observations{_observations} {
	}
};

inline bool operator==(const Landmark &lhs, const Landmark &rhs) {
	return (lhs.pt == rhs.pt) && (lhs.id == rhs.id) && (lhs.timestamp == rhs.timestamp)
		&& (lhs.descriptor == rhs.descriptor) && (lhs.descriptorType == rhs.descriptorType)
		&& (lhs.covariance == rhs.covariance) && (lhs.observations == rhs.observations);
}

struct LandmarkFlatbuffer FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
	typedef Landmark NativeTableType;

	static const flatbuffers::TypeTable *MiniReflectTypeTable() {
		return LandmarkTypeTable();
	}

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.Landmark";
	}

	enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
		VT_PT             = 4,
		VT_ID             = 6,
		VT_TIMESTAMP      = 8,
		VT_DESCRIPTOR     = 10,
		VT_DESCRIPTORTYPE = 12,
		VT_COVARIANCE     = 14,
		VT_OBSERVATIONS   = 16
	};

	/// 3D coordinate of the landmark.
	const Point3f *pt() const {
		return GetStruct<const Point3f *>(VT_PT);
	}

	/// Landmark id (if the keypoints need to be clustered by an object they belong to).
	int64_t id() const {
		return GetField<int64_t>(VT_ID, 0);
	}

	/// Timestamp (µs).
	int64_t timestamp() const {
		return GetField<int64_t>(VT_TIMESTAMP, 0);
	}

	/// Visual descriptor of the landmark.
	const flatbuffers::Vector<int8_t> *descriptor() const {
		return GetPointer<const flatbuffers::Vector<int8_t> *>(VT_DESCRIPTOR);
	}

	/// Type of the visual descriptor
	const flatbuffers::String *descriptorType() const {
		return GetPointer<const flatbuffers::String *>(VT_DESCRIPTORTYPE);
	}

	/// Covariance matrix, must contain 9 numbers. It is represented as a 3x3 square matrix.
	const flatbuffers::Vector<float> *covariance() const {
		return GetPointer<const flatbuffers::Vector<float> *>(VT_COVARIANCE);
	}

	/// Observation info, can be from multiple cameras if they are matched using descriptor.
	const flatbuffers::Vector<flatbuffers::Offset<ObservationFlatbuffer>> *observations() const {
		return GetPointer<const flatbuffers::Vector<flatbuffers::Offset<ObservationFlatbuffer>> *>(VT_OBSERVATIONS);
	}

	bool Verify(flatbuffers::Verifier &verifier) const {
		return VerifyTableStart(verifier) && VerifyField<Point3f>(verifier, VT_PT)
			&& VerifyField<int64_t>(verifier, VT_ID) && VerifyField<int64_t>(verifier, VT_TIMESTAMP)
			&& VerifyOffset(verifier, VT_DESCRIPTOR) && verifier.VerifyVector(descriptor())
			&& VerifyOffset(verifier, VT_DESCRIPTORTYPE) && verifier.VerifyString(descriptorType())
			&& VerifyOffset(verifier, VT_COVARIANCE) && verifier.VerifyVector(covariance())
			&& VerifyOffset(verifier, VT_OBSERVATIONS) && verifier.VerifyVector(observations())
			&& verifier.VerifyVectorOfTables(observations()) && verifier.EndTable();
	}

	Landmark *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	void UnPackTo(Landmark *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	static void UnPackToFrom(
		Landmark *_o, const LandmarkFlatbuffer *_fb, const flatbuffers::resolver_function_t *_resolver = nullptr);
	static flatbuffers::Offset<LandmarkFlatbuffer> Pack(flatbuffers::FlatBufferBuilder &_fbb, const Landmark *_o,
		const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct LandmarkBuilder {
	flatbuffers::FlatBufferBuilder &fbb_;
	flatbuffers::uoffset_t start_;

	void add_pt(const Point3f *pt) {
		fbb_.AddStruct(LandmarkFlatbuffer::VT_PT, pt);
	}

	void add_id(int64_t id) {
		fbb_.AddElement<int64_t>(LandmarkFlatbuffer::VT_ID, id, 0);
	}

	void add_timestamp(int64_t timestamp) {
		fbb_.AddElement<int64_t>(LandmarkFlatbuffer::VT_TIMESTAMP, timestamp, 0);
	}

	void add_descriptor(flatbuffers::Offset<flatbuffers::Vector<int8_t>> descriptor) {
		fbb_.AddOffset(LandmarkFlatbuffer::VT_DESCRIPTOR, descriptor);
	}

	void add_descriptorType(flatbuffers::Offset<flatbuffers::String> descriptorType) {
		fbb_.AddOffset(LandmarkFlatbuffer::VT_DESCRIPTORTYPE, descriptorType);
	}

	void add_covariance(flatbuffers::Offset<flatbuffers::Vector<float>> covariance) {
		fbb_.AddOffset(LandmarkFlatbuffer::VT_COVARIANCE, covariance);
	}

	void add_observations(
		flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<ObservationFlatbuffer>>> observations) {
		fbb_.AddOffset(LandmarkFlatbuffer::VT_OBSERVATIONS, observations);
	}

	explicit LandmarkBuilder(flatbuffers::FlatBufferBuilder &_fbb) : fbb_(_fbb) {
		start_ = fbb_.StartTable();
	}

	LandmarkBuilder &operator=(const LandmarkBuilder &);

	flatbuffers::Offset<LandmarkFlatbuffer> Finish() {
		const auto end = fbb_.EndTable(start_);
		auto o         = flatbuffers::Offset<LandmarkFlatbuffer>(end);
		return o;
	}
};

inline flatbuffers::Offset<LandmarkFlatbuffer> CreateLandmark(flatbuffers::FlatBufferBuilder &_fbb,
	const Point3f *pt = 0, int64_t id = 0, int64_t timestamp = 0,
	flatbuffers::Offset<flatbuffers::Vector<int8_t>> descriptor                                       = 0,
	flatbuffers::Offset<flatbuffers::String> descriptorType                                           = 0,
	flatbuffers::Offset<flatbuffers::Vector<float>> covariance                                        = 0,
	flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<ObservationFlatbuffer>>> observations = 0) {
	LandmarkBuilder builder_(_fbb);
	builder_.add_timestamp(timestamp);
	builder_.add_id(id);
	builder_.add_observations(observations);
	builder_.add_covariance(covariance);
	builder_.add_descriptorType(descriptorType);
	builder_.add_descriptor(descriptor);
	builder_.add_pt(pt);
	return builder_.Finish();
}

inline flatbuffers::Offset<LandmarkFlatbuffer> CreateLandmarkDirect(flatbuffers::FlatBufferBuilder &_fbb,
	const Point3f *pt = 0, int64_t id = 0, int64_t timestamp = 0, const std::vector<int8_t> *descriptor = nullptr,
	const char *descriptorType = nullptr, const std::vector<float> *covariance = nullptr,
	const std::vector<flatbuffers::Offset<ObservationFlatbuffer>> *observations = nullptr) {
	auto descriptor__     = descriptor ? _fbb.CreateVector<int8_t>(*descriptor) : 0;
	auto descriptorType__ = descriptorType ? _fbb.CreateString(descriptorType) : 0;
	auto covariance__     = covariance ? _fbb.CreateVector<float>(*covariance) : 0;
	auto observations__
		= observations ? _fbb.CreateVector<flatbuffers::Offset<ObservationFlatbuffer>>(*observations) : 0;
	return dv::CreateLandmark(_fbb, pt, id, timestamp, descriptor__, descriptorType__, covariance__, observations__);
}

flatbuffers::Offset<LandmarkFlatbuffer> CreateLandmark(flatbuffers::FlatBufferBuilder &_fbb, const Landmark *_o,
	const flatbuffers::rehasher_function_t *_rehasher = nullptr);

struct LandmarksPacket : public flatbuffers::NativeTable {
	typedef LandmarksPacketFlatbuffer TableType;

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.LandmarksPacket";
	}

	dv::cvector<Landmark> elements;
	dv::cstring referenceFrame;

	LandmarksPacket() {
	}

	// Generated Constructor
	LandmarksPacket(const dv::cvector<Landmark> &_elements, const dv::cstring &_referenceFrame) :
		elements{_elements},
		referenceFrame{_referenceFrame} {
	}

	friend std::ostream &operator<<(std::ostream &os, const LandmarksPacket &packet) {
		if (packet.elements.empty()) {
			os << fmt::format("LandmarksPacket containing 0 landmarks");
			return os;
		}

		const int64_t lowestTime  = packet.elements.front().timestamp;
		const int64_t highestTime = packet.elements.back().timestamp;

		os << fmt::format("LandmarksPacket containing {} landmarks within {}μs duration; time range within [{}; {}]",
			packet.elements.size(), highestTime - lowestTime, lowestTime, highestTime);
		return os;
	}
};

inline bool operator==(const LandmarksPacket &lhs, const LandmarksPacket &rhs) {
	return (lhs.elements == rhs.elements) && (lhs.referenceFrame == rhs.referenceFrame);
}

struct LandmarksPacketFlatbuffer FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
	typedef LandmarksPacket NativeTableType;
	static FLATBUFFERS_CONSTEXPR const char *identifier = "LMRS";

	static const flatbuffers::TypeTable *MiniReflectTypeTable() {
		return LandmarksPacketTypeTable();
	}

	static FLATBUFFERS_CONSTEXPR const char *GetFullyQualifiedName() {
		return "dv.LandmarksPacket";
	}

	enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
		VT_ELEMENTS       = 4,
		VT_REFERENCEFRAME = 6
	};

	const flatbuffers::Vector<flatbuffers::Offset<LandmarkFlatbuffer>> *elements() const {
		return GetPointer<const flatbuffers::Vector<flatbuffers::Offset<LandmarkFlatbuffer>> *>(VT_ELEMENTS);
	}

	/// Coordinate reference frame of the landmarks, "world" coordinate frame by default
	const flatbuffers::String *referenceFrame() const {
		return GetPointer<const flatbuffers::String *>(VT_REFERENCEFRAME);
	}

	bool Verify(flatbuffers::Verifier &verifier) const {
		return VerifyTableStart(verifier) && VerifyOffset(verifier, VT_ELEMENTS) && verifier.VerifyVector(elements())
			&& verifier.VerifyVectorOfTables(elements()) && VerifyOffset(verifier, VT_REFERENCEFRAME)
			&& verifier.VerifyString(referenceFrame()) && verifier.EndTable();
	}

	LandmarksPacket *UnPack(const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	void UnPackTo(LandmarksPacket *_o, const flatbuffers::resolver_function_t *_resolver = nullptr) const;
	static void UnPackToFrom(LandmarksPacket *_o, const LandmarksPacketFlatbuffer *_fb,
		const flatbuffers::resolver_function_t *_resolver = nullptr);
	static flatbuffers::Offset<LandmarksPacketFlatbuffer> Pack(flatbuffers::FlatBufferBuilder &_fbb,
		const LandmarksPacket *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);
};

struct LandmarksPacketBuilder {
	flatbuffers::FlatBufferBuilder &fbb_;
	flatbuffers::uoffset_t start_;

	void add_elements(flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<LandmarkFlatbuffer>>> elements) {
		fbb_.AddOffset(LandmarksPacketFlatbuffer::VT_ELEMENTS, elements);
	}

	void add_referenceFrame(flatbuffers::Offset<flatbuffers::String> referenceFrame) {
		fbb_.AddOffset(LandmarksPacketFlatbuffer::VT_REFERENCEFRAME, referenceFrame);
	}

	explicit LandmarksPacketBuilder(flatbuffers::FlatBufferBuilder &_fbb) : fbb_(_fbb) {
		start_ = fbb_.StartTable();
	}

	LandmarksPacketBuilder &operator=(const LandmarksPacketBuilder &);

	flatbuffers::Offset<LandmarksPacketFlatbuffer> Finish() {
		const auto end = fbb_.EndTable(start_);
		auto o         = flatbuffers::Offset<LandmarksPacketFlatbuffer>(end);
		return o;
	}
};

inline flatbuffers::Offset<LandmarksPacketFlatbuffer> CreateLandmarksPacket(flatbuffers::FlatBufferBuilder &_fbb,
	flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<LandmarkFlatbuffer>>> elements = 0,
	flatbuffers::Offset<flatbuffers::String> referenceFrame                                    = 0) {
	LandmarksPacketBuilder builder_(_fbb);
	builder_.add_referenceFrame(referenceFrame);
	builder_.add_elements(elements);
	return builder_.Finish();
}

inline flatbuffers::Offset<LandmarksPacketFlatbuffer> CreateLandmarksPacketDirect(flatbuffers::FlatBufferBuilder &_fbb,
	const std::vector<flatbuffers::Offset<LandmarkFlatbuffer>> *elements = nullptr,
	const char *referenceFrame                                           = nullptr) {
	auto elements__       = elements ? _fbb.CreateVector<flatbuffers::Offset<LandmarkFlatbuffer>>(*elements) : 0;
	auto referenceFrame__ = referenceFrame ? _fbb.CreateString(referenceFrame) : 0;
	return dv::CreateLandmarksPacket(_fbb, elements__, referenceFrame__);
}

flatbuffers::Offset<LandmarksPacketFlatbuffer> CreateLandmarksPacket(flatbuffers::FlatBufferBuilder &_fbb,
	const LandmarksPacket *_o, const flatbuffers::rehasher_function_t *_rehasher = nullptr);

inline Observation *ObservationFlatbuffer::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
	auto _o = new Observation();
	UnPackTo(_o, _resolver);
	return _o;
}

inline void ObservationFlatbuffer::UnPackTo(Observation *_o, const flatbuffers::resolver_function_t *_resolver) const {
	(void) _o;
	(void) _resolver;
	UnPackToFrom(_o, this, _resolver);
}

inline void ObservationFlatbuffer::UnPackToFrom(
	Observation *_o, const ObservationFlatbuffer *_fb, const flatbuffers::resolver_function_t *_resolver) {
	(void) _o;
	(void) _fb;
	(void) _resolver;
	{
		auto _e     = _fb->trackId();
		_o->trackId = _e;
	};
	{
		auto _e      = _fb->cameraId();
		_o->cameraId = _e;
	};
	{
		auto _e = _fb->cameraName();
		if (_e)
			_o->cameraName = dv::cstring(_e->c_str(), _e->size());
	};
	{
		auto _e       = _fb->timestamp();
		_o->timestamp = _e;
	};
}

inline flatbuffers::Offset<ObservationFlatbuffer> ObservationFlatbuffer::Pack(
	flatbuffers::FlatBufferBuilder &_fbb, const Observation *_o, const flatbuffers::rehasher_function_t *_rehasher) {
	return CreateObservation(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<ObservationFlatbuffer> CreateObservation(
	flatbuffers::FlatBufferBuilder &_fbb, const Observation *_o, const flatbuffers::rehasher_function_t *_rehasher) {
	(void) _rehasher;
	(void) _o;

	struct _VectorArgs {
		flatbuffers::FlatBufferBuilder *__fbb;
		const Observation *__o;
		const flatbuffers::rehasher_function_t *__rehasher;
	} _va = {&_fbb, _o, _rehasher};

	(void) _va;
	auto _trackId    = _o->trackId;
	auto _cameraId   = _o->cameraId;
	auto _cameraName = _o->cameraName.empty() ? 0 : _fbb.CreateString(_o->cameraName);
	auto _timestamp  = _o->timestamp;
	return dv::CreateObservation(_fbb, _trackId, _cameraId, _cameraName, _timestamp);
}

inline Landmark *LandmarkFlatbuffer::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
	auto _o = new Landmark();
	UnPackTo(_o, _resolver);
	return _o;
}

inline void LandmarkFlatbuffer::UnPackTo(Landmark *_o, const flatbuffers::resolver_function_t *_resolver) const {
	(void) _o;
	(void) _resolver;
	UnPackToFrom(_o, this, _resolver);
}

inline void LandmarkFlatbuffer::UnPackToFrom(
	Landmark *_o, const LandmarkFlatbuffer *_fb, const flatbuffers::resolver_function_t *_resolver) {
	(void) _o;
	(void) _fb;
	(void) _resolver;
	{
		auto _e = _fb->pt();
		if (_e)
			_o->pt = *_e;
	};
	{
		auto _e = _fb->id();
		_o->id  = _e;
	};
	{
		auto _e       = _fb->timestamp();
		_o->timestamp = _e;
	};
	{
		auto _e = _fb->descriptor();
		if (_e) {
			_o->descriptor.resize(_e->size());
			for (flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) {
				_o->descriptor[_i] = _e->Get(_i);
			}
		}
	};
	{
		auto _e = _fb->descriptorType();
		if (_e)
			_o->descriptorType = dv::cstring(_e->c_str(), _e->size());
	};
	{
		auto _e = _fb->covariance();
		if (_e) {
			_o->covariance.resize(_e->size());
			for (flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) {
				_o->covariance[_i] = _e->Get(_i);
			}
		}
	};
	{
		auto _e = _fb->observations();
		if (_e) {
			_o->observations.resize(_e->size());
			for (flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) {
				_e->Get(_i)->UnPackTo(&_o->observations[_i], _resolver);
			}
		}
	};
}

inline flatbuffers::Offset<LandmarkFlatbuffer> LandmarkFlatbuffer::Pack(
	flatbuffers::FlatBufferBuilder &_fbb, const Landmark *_o, const flatbuffers::rehasher_function_t *_rehasher) {
	return CreateLandmark(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<LandmarkFlatbuffer> CreateLandmark(
	flatbuffers::FlatBufferBuilder &_fbb, const Landmark *_o, const flatbuffers::rehasher_function_t *_rehasher) {
	(void) _rehasher;
	(void) _o;

	struct _VectorArgs {
		flatbuffers::FlatBufferBuilder *__fbb;
		const Landmark *__o;
		const flatbuffers::rehasher_function_t *__rehasher;
	} _va = {&_fbb, _o, _rehasher};

	(void) _va;
	auto _pt             = &_o->pt;
	auto _id             = _o->id;
	auto _timestamp      = _o->timestamp;
	auto _descriptor     = _o->descriptor.size() ? _fbb.CreateVector(_o->descriptor.data(), _o->descriptor.size()) : 0;
	auto _descriptorType = _o->descriptorType.empty() ? 0 : _fbb.CreateString(_o->descriptorType);
	auto _covariance     = _o->covariance.size() ? _fbb.CreateVector(_o->covariance.data(), _o->covariance.size()) : 0;
	auto _observations   = _o->observations.size() ? _fbb.CreateVector<flatbuffers::Offset<ObservationFlatbuffer>>(
                             _o->observations.size(),
                             [](size_t i, _VectorArgs *__va) {
                                 return CreateObservation(*__va->__fbb, &__va->__o->observations[i], __va->__rehasher);
                             },
                             &_va)
												   : 0;
	return dv::CreateLandmark(_fbb, _pt, _id, _timestamp, _descriptor, _descriptorType, _covariance, _observations);
}

inline LandmarksPacket *LandmarksPacketFlatbuffer::UnPack(const flatbuffers::resolver_function_t *_resolver) const {
	auto _o = new LandmarksPacket();
	UnPackTo(_o, _resolver);
	return _o;
}

inline void LandmarksPacketFlatbuffer::UnPackTo(
	LandmarksPacket *_o, const flatbuffers::resolver_function_t *_resolver) const {
	(void) _o;
	(void) _resolver;
	UnPackToFrom(_o, this, _resolver);
}

inline void LandmarksPacketFlatbuffer::UnPackToFrom(
	LandmarksPacket *_o, const LandmarksPacketFlatbuffer *_fb, const flatbuffers::resolver_function_t *_resolver) {
	(void) _o;
	(void) _fb;
	(void) _resolver;
	{
		auto _e = _fb->elements();
		if (_e) {
			_o->elements.resize(_e->size());
			for (flatbuffers::uoffset_t _i = 0; _i < _e->size(); _i++) {
				_e->Get(_i)->UnPackTo(&_o->elements[_i], _resolver);
			}
		}
	};
	{
		auto _e = _fb->referenceFrame();
		if (_e)
			_o->referenceFrame = dv::cstring(_e->c_str(), _e->size());
	};
}

inline flatbuffers::Offset<LandmarksPacketFlatbuffer> LandmarksPacketFlatbuffer::Pack(
	flatbuffers::FlatBufferBuilder &_fbb, const LandmarksPacket *_o,
	const flatbuffers::rehasher_function_t *_rehasher) {
	return CreateLandmarksPacket(_fbb, _o, _rehasher);
}

inline flatbuffers::Offset<LandmarksPacketFlatbuffer> CreateLandmarksPacket(flatbuffers::FlatBufferBuilder &_fbb,
	const LandmarksPacket *_o, const flatbuffers::rehasher_function_t *_rehasher) {
	(void) _rehasher;
	(void) _o;

	struct _VectorArgs {
		flatbuffers::FlatBufferBuilder *__fbb;
		const LandmarksPacket *__o;
		const flatbuffers::rehasher_function_t *__rehasher;
	} _va = {&_fbb, _o, _rehasher};

	(void) _va;
	auto _elements       = _o->elements.size() ? _fbb.CreateVector<flatbuffers::Offset<LandmarkFlatbuffer>>(
                         _o->elements.size(),
                         [](size_t i, _VectorArgs *__va) {
                             return CreateLandmark(*__va->__fbb, &__va->__o->elements[i], __va->__rehasher);
                         },
                         &_va)
											   : 0;
	auto _referenceFrame = _o->referenceFrame.empty() ? 0 : _fbb.CreateString(_o->referenceFrame);
	return dv::CreateLandmarksPacket(_fbb, _elements, _referenceFrame);
}

inline const flatbuffers::TypeTable *ObservationTypeTable() {
	static const flatbuffers::TypeCode type_codes[] = {
		{flatbuffers::ET_INT,    0, -1},
        {flatbuffers::ET_INT,    0, -1},
        {flatbuffers::ET_STRING, 0, -1},
		{flatbuffers::ET_LONG,   0, -1}
    };
	static const char *const names[]       = {"trackId", "cameraId", "cameraName", "timestamp"};
	static const flatbuffers::TypeTable tt = {flatbuffers::ST_TABLE, 4, type_codes, nullptr, nullptr, names};
	return &tt;
}

inline const flatbuffers::TypeTable *LandmarkTypeTable() {
	static const flatbuffers::TypeCode type_codes[] = {
		{flatbuffers::ET_SEQUENCE, 0, 0 },
        {flatbuffers::ET_LONG,     0, -1},
        {flatbuffers::ET_LONG,     0, -1},
		{flatbuffers::ET_CHAR,     1, -1},
        {flatbuffers::ET_STRING,   0, -1},
        {flatbuffers::ET_FLOAT,    1, -1},
		{flatbuffers::ET_SEQUENCE, 1, 1 }
    };
	static const flatbuffers::TypeFunction type_refs[] = {Point3fTypeTable, ObservationTypeTable};
	static const char *const names[]
		= {"pt", "id", "timestamp", "descriptor", "descriptorType", "covariance", "observations"};
	static const flatbuffers::TypeTable tt = {flatbuffers::ST_TABLE, 7, type_codes, type_refs, nullptr, names};
	return &tt;
}

inline const flatbuffers::TypeTable *LandmarksPacketTypeTable() {
	static const flatbuffers::TypeCode type_codes[] = {
		{flatbuffers::ET_SEQUENCE, 1, 0 },
        {flatbuffers::ET_STRING,   0, -1}
    };
	static const flatbuffers::TypeFunction type_refs[] = {LandmarkTypeTable};
	static const char *const names[]                   = {"elements", "referenceFrame"};
	static const flatbuffers::TypeTable tt = {flatbuffers::ST_TABLE, 2, type_codes, type_refs, nullptr, names};
	return &tt;
}

inline const dv::LandmarksPacketFlatbuffer *GetLandmarksPacket(const void *buf) {
	return flatbuffers::GetRoot<dv::LandmarksPacketFlatbuffer>(buf);
}

inline const dv::LandmarksPacketFlatbuffer *GetSizePrefixedLandmarksPacket(const void *buf) {
	return flatbuffers::GetSizePrefixedRoot<dv::LandmarksPacketFlatbuffer>(buf);
}

inline const char *LandmarksPacketIdentifier() {
	return "LMRS";
}

inline bool LandmarksPacketBufferHasIdentifier(const void *buf) {
	return flatbuffers::BufferHasIdentifier(buf, LandmarksPacketIdentifier());
}

inline bool VerifyLandmarksPacketBuffer(flatbuffers::Verifier &verifier) {
	return verifier.VerifyBuffer<dv::LandmarksPacketFlatbuffer>(LandmarksPacketIdentifier());
}

inline bool VerifySizePrefixedLandmarksPacketBuffer(flatbuffers::Verifier &verifier) {
	return verifier.VerifySizePrefixedBuffer<dv::LandmarksPacketFlatbuffer>(LandmarksPacketIdentifier());
}

inline void FinishLandmarksPacketBuffer(
	flatbuffers::FlatBufferBuilder &fbb, flatbuffers::Offset<dv::LandmarksPacketFlatbuffer> root) {
	fbb.Finish(root, LandmarksPacketIdentifier());
}

inline void FinishSizePrefixedLandmarksPacketBuffer(
	flatbuffers::FlatBufferBuilder &fbb, flatbuffers::Offset<dv::LandmarksPacketFlatbuffer> root) {
	fbb.FinishSizePrefixed(root, LandmarksPacketIdentifier());
}

inline std::unique_ptr<LandmarksPacket> UnPackLandmarksPacket(
	const void *buf, const flatbuffers::resolver_function_t *res = nullptr) {
	return std::unique_ptr<LandmarksPacket>(GetLandmarksPacket(buf)->UnPack(res));
}

} // namespace dv

// fmt compatibility for ostream class printing.
template<>
struct fmt::formatter<dv::LandmarksPacket> : fmt::ostream_formatter {};

#endif // FLATBUFFERS_GENERATED_LANDMARK_DV_H_