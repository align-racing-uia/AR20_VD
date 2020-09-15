FNBH_vektor=[18.04 33.28 0.31]
FNFH_vektor=[15.53 30.81 0.12]
BNFV_vektor=[14.63 7.23 2.12]
BOFH_vektor=[17.96 7.2 0.81]
FOFH_vektor=[15.47 27.19 1.29]
xy_norm=[0 0 1]
yz_norm=[1 0 0]
FNBH_xy_angle = 90-atan2d(norm(cross(FNBH_vektor,xy_norm)),dot(FNBH_vektor,xy_norm))
FNBH_yz_angle = 90-atan2d(norm(cross(FNBH_vektor,yz_norm)),dot(FNBH_vektor,yz_norm))

FNFH_xy_angle = 90-atan2d(norm(cross(FNFH_vektor,xy_norm)),dot(FNFH_vektor,xy_norm))
FNFH_yz_angle = 90-atan2d(norm(cross(FNFH_vektor,yz_norm)),dot(FNFH_vektor,yz_norm))

BNFV_xy_angle = 90-atan2d(norm(cross(BNFV_vektor,xy_norm)),dot(BNFV_vektor,xy_norm))
BNFV_yz_angle = 90-atan2d(norm(cross(BNFV_vektor,yz_norm)),dot(BNFV_vektor,yz_norm))

BOFH_xy_angle = 90-atan2d(norm(cross(BOFH_vektor,xy_norm)),dot(BOFH_vektor,xy_norm))
BOFH_yz_angle = 90-atan2d(norm(cross(BOFH_vektor,yz_norm)),dot(BOFH_vektor,yz_norm))

FOFH_xy_angle = 90-atan2d(norm(cross(FOFH_vektor,xy_norm)),dot(FOFH_vektor,xy_norm))
FOFH_yz_angle = 90-atan2d(norm(cross(FOFH_vektor,yz_norm)),dot(FOFH_vektor,yz_norm))