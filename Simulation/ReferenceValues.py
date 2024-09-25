import numpy as np
from scipy.spatial.transform import Rotation
import quaternion as quat
from Simulation.SimulationMath import vecNormalize, vectorUp, rotationToVectorFromBase

gravityConstant = 6.67e-11
earthAccelerationFreeFall = 9.80665;

earthName = "Earth"
earthRadius = 6378000
earthMass = 5.972e24
earthPosition = 149e9 * np.array([1, 0, 0])
earthVelocity = 29766 * np.array([0, 1, 0])
earthAxis = vecNormalize(np.cross(earthVelocity, earthPosition))
earthRotation = Rotation.from_rotvec(23.5 * vecNormalize(earthPosition), degrees=True) * rotationToVectorFromBase(earthAxis)
earthRotationSpeed = Rotation.from_rotvec(earthAxis * 360 / (24 * 60 * 60), degrees=True)

sunName = "Sun"
sunRadius = 696340000
sunMass = 1.989e30
sunPosition = np.array([0, 0, 0])

marsName = "Mars"
marsRadius = 3390000
marsMass = 6.39e23
marsPosition = 229e9 * np.array([1, 0, 0])
marsVelocity = 24000 * np.array([0, 1, 0])
marsAxis = vecNormalize(np.cross(marsVelocity, marsPosition))
marsRotation = Rotation.from_rotvec(25.2 * vecNormalize(marsPosition), degrees=True) * rotationToVectorFromBase(marsAxis)
marsRotationSpeed = Rotation.from_rotvec(marsAxis * 360 / (24 * 60 * 60) / 1.02569, degrees=True)

# based on https://ru.wikipedia.org/wiki/%D0%9A%D0%BE%D1%81%D0%BC%D0%BE%D1%81_(%D1%81%D0%B5%D0%BC%D0%B5%D0%B9%D1%81%D1%82%D0%B2%D0%BE_%D1%80%D0%B0%D0%BA%D0%B5%D1%82-%D0%BD%D0%BE%D1%81%D0%B8%D1%82%D0%B5%D0%BB%D0%B5%D0%B9)
quaternion = quat.as_float_array(quat.from_spherical_coords(np.deg2rad(90), 0))

rocketName = "Rocket"
rocketMass = 109000
rocketVolume = 146.5
rocketMaxForce = 1486000
rocketVelocity = earthVelocity
rocketRotation = earthRotation * Rotation.from_quat(quaternion)
rocketPosition = earthPosition + (rocketRotation).apply(vectorUp) * earthRadius
rocketRadius = 1.2

# based on https://ru.wikipedia.org/wiki/%D0%9C%D0%B5%D0%B6%D0%B4%D1%83%D0%BD%D0%B0%D1%80%D0%BE%D0%B4%D0%BD%D0%B0%D1%8F_%D0%BA%D0%BE%D1%81%D0%BC%D0%B8%D1%87%D0%B5%D1%81%D0%BA%D0%B0%D1%8F_%D1%81%D1%82%D0%B0%D0%BD%D1%86%D0%B8%D1%8F
mksName = "MKS"
mksMass = 440000
mksFlyHeight = 418000
mksRotationProjected = earthRotation * Rotation.from_quat(quaternion)
mksVelocity = mksRotationProjected.apply(np.array([0, 1, 0])) * 7700 + earthVelocity
mksPosition = earthPosition + (mksRotationProjected).apply(vectorUp) * (earthRadius + mksFlyHeight)