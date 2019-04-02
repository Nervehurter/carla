// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/RayCastLidar.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "StaticMeshResources.h"

FActorDefinition ARayCastLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));
}

ARayCastLidar::ARayCastLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;

  auto MeshComp = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RootComponent"));
  MeshComp->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
  MeshComp->bHiddenInGame = true;
  MeshComp->CastShadow = false;
  MeshComp->PostPhysicsComponentTick.bCanEverTick = false;
  RootComponent = MeshComp;
}

void ARayCastLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LidarMeasurement = FLidarMeasurement(Description.Channels);
  CreateLasers();
}

void ARayCastLidar::CreateLasers()
{
  // check if inputs are correct
  const auto NumberOfLasers = Description.Channels;
  check(NumberOfLasers == Description.layer_elevation.Num());
  check(NumberOfLasers == Description.layer_azimuth.Num());

  // fill laser properties
  RangeStd = Description.RangeMeasStd;
  LaserElevationAngles.Empty(NumberOfLasers);
  LaserAzimuthOffsets.Empty(NumberOfLasers);
  for(auto i = 0u; i < NumberOfLasers; ++i)
  {
    const float VerticalAngle = Description.layer_elevation[i];
    const float AzimuthOffset = Description.layer_azimuth[i];
    LaserElevationAngles.Emplace(VerticalAngle);
    LaserAzimuthOffsets.Emplace(AzimuthOffset);
  }
}

void ARayCastLidar::Tick(const float DeltaTime)
{
  // increment time read and send points
  Super::Tick(DeltaTime);

  ReadPoints(DeltaTime);

  auto DataStream = GetDataStream(*this);
  DataStream.Send(*this, LidarMeasurement, DataStream.PopBufferFromPool());
}

void ARayCastLidar::ReadPoints(const float DeltaTime)
{
  //TODO: improve readability by using angular resolution
  const uint32 ChannelCount = Description.Channels;
  const uint32 PointsToScanWithOneLaser =
    FMath::RoundHalfFromZero(
        Description.PointsPerSecond * DeltaTime / float(ChannelCount));

  if (PointsToScanWithOneLaser <= 0)
  {
    UE_LOG(
        LogCarla,
        Warning,
        TEXT("%s: no points requested this frame, try increasing the number of points per second."),
        *GetName());
    return;
  }

  // check properties
  check(ChannelCount == LaserElevationAngles.Num());
  check(ChannelCount == LaserAzimuthOffsets.Num());

  // calc horizontal angular distance between points in one layer
  const float CurrentHorizontalAngle = LidarMeasurement.GetHorizontalAngle();
  const float AngleDistanceOfTick = Description.RotationFrequency * 360.0f * DeltaTime;
  const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

  LidarMeasurement.Reset(ChannelCount * PointsToScanWithOneLaser);

  // loop over channels
  for (auto Channel = 0u; Channel < ChannelCount; ++Channel)
  {
    // loop over points in channel that are recorded in tick
    for (auto i = 0u; i < PointsToScanWithOneLaser; ++i)
    {
      FVector Point;
      const float Angle = CurrentHorizontalAngle + AngleDistanceOfLaserMeasure * i;
      if (ShootLaser(Channel, Angle, Point))
      {
        LidarMeasurement.WritePoint(Channel, Point);
      }
    }
  }

  const float HorizontalAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, 360.0f);
  LidarMeasurement.SetHorizontalAngle(HorizontalAngle);
}

bool ARayCastLidar::ShootLaser(const uint32 Channel, const float HorizontalAngle, FVector &XYZ) const
{
  // get elevation and azimuth for laser beam
  const float ElevationAngle = LaserElevationAngles[Channel];
  const float AzimuthAngle = HorizontalAngle + LaserAzimuthOffsets[Channel];

  FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
  TraceParams.bTraceComplex = true;
  TraceParams.bReturnPhysicalMaterial = false;

  FHitResult HitInfo(ForceInit);

  // get rotation
  FVector LidarBodyLoc = GetActorLocation();
  FRotator LidarBodyRot = GetActorRotation();
  FRotator LaserRot (ElevationAngle, AzimuthAngle, 0);  // float InPitch, float InYaw, float InRoll
  FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
    LaserRot,
    LidarBodyRot
  );
  const auto Range = Description.Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  // do ray tracing
  GetWorld()->LineTraceSingleByChannel(
    HitInfo,
    LidarBodyLoc,
    EndTrace,
    ECC_MAX,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam
  );

  if (HitInfo.bBlockingHit)
  {
    // laser beam hit something
    if (Description.ShowDebugPoints)
    {
      DrawDebugPoint(
        GetWorld(),
        HitInfo.ImpactPoint,
        10,  //size
        FColor(255,0,255),
        false,  //persistent (never goes away)
        0.1  //point leaves a trail on moving object
      );
    }

    // ground truth of measurement point
    FVector gt_XYZ = LidarBodyLoc - HitInfo.ImpactPoint;

    // apply range measurement uncertainty
    const float range_error = FMath::RandRange(-1.0f, 1.0f) * RangeStd;
    const float range = gt_XYZ.Size();
    XYZ = (gt_XYZ / range) * (range + range_error);

    // make 0 deg facing front (instead right)
    XYZ = UKismetMathLibrary::RotateAngleAxis(
      XYZ,
      - LidarBodyRot.Yaw + 90,
      FVector(0, 0, 1)
    );

    return true;
  } else {
    // laser beam did not hit anything
    return false;
  }
}
