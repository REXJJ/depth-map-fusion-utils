#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>

namespace TransformationUtilities
{
    inline Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
    {
        Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
        Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
        data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
        data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
        Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
        Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
        transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
        return transformed_data_mat.transpose();
    }

    template<typename PointT> void transformPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud_input,typename pcl::PointCloud<PointT>::Ptr cloud_output,Eigen::MatrixXd& a_T_b)
    {
        for(auto point:cloud_input->points)
        {
            Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
            pts(0,0)=point.x;
            pts(0,1)=point.y;
            pts(0,2)=point.z;
            Eigen::MatrixXd pts_trans=apply_transformation(pts,a_T_b);
            auto pt = point;
            pt.x=pts_trans(0,0);
            pt.y=pts_trans(0,1);
            pt.z=pts_trans(0,2);
            cloud_output->points.push_back(pt);
        }
    }
    
    void transformPointCloudNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud_input,pcl::PointCloud<pcl::Normal>::Ptr cloud_output,Eigen::MatrixXd& a_T_b)
    {
        for(auto point:cloud_input->points)
        {
            Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
            pts(0,0)=point.normal[0];
            pts(0,1)=point.normal[1];
            pts(0,2)=point.normal[2];
            Eigen::MatrixXd rotation = a_T_b;
            rotation(3,0)=0;
            rotation(3,1)=0;
            rotation(3,2)=0;
            Eigen::MatrixXd pts_trans=apply_transformation(pts,rotation);
            auto pt = point;
            pt.normal[0]=pts_trans(0,0);
            pt.normal[1]=pts_trans(0,1);
            pt.normal[2]=pts_trans(0,2);
            cloud_output->points.push_back(pt);
        }
    }
    
    std::string validate_seq(std::string seq)
	{
		if(seq =="")
			seq = "ZYX";	
		bool invalid_flag = false;
		if(seq.size()!=3)
		{
			invalid_flag = true;
		}
		for (int i =0;i<3;++i)
			if(seq[i]!='X' && seq[i]!='Y' && seq[i]!='Z' && seq[i]!='x' && seq[i]!='y' && seq[i]!='z')
			{
				invalid_flag = true; 
				break;
			}
		if(invalid_flag)
		{
			std::cerr << "ERROR: Invalid Rotations Sequence: " << seq << std::endl;
			std::terminate();		
		}
		return seq;
	}

    //Default : ZYX
    Eigen::Matrix3d eul2rot(Eigen::MatrixXd eul_angles, std::string seq="")
    {
        seq = validate_seq(seq);
        Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
        for (int i=0; i<3; ++i)
        {
            if(seq[i]=='X' || seq[i]=='x')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitX());
            else if(seq[i]=='Y' || seq[i]=='y')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitY());			
            else if(seq[i]=='Z' || seq[i]=='z')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitZ());					
        }
        return rot_mat; 
    }

    Eigen::MatrixXd vectorToTransformationMatrix(vector<double>& a_T_b_static)
    {
        Eigen::MatrixXd a_T_b = Eigen::MatrixXd::Identity(4,4);
        a_T_b(0,3) = a_T_b_static[0];        
        a_T_b(1,3) = a_T_b_static[1];
        a_T_b(2,3) = a_T_b_static[2];
        Eigen::MatrixXd eul_ang(1,3);
        eul_ang<<a_T_b_static[3],a_T_b_static[4],a_T_b_static[5];
        a_T_b.block(0,0,3,3) = eul2rot(eul_ang);
        return a_T_b;
    }
    
    Eigen::Affine3f vectorToAffineMatrix(vector<double> location)
    {
        Eigen::MatrixXd two_T_one = vectorToTransformationMatrix(location);
        Eigen::Affine3f transformation = Eigen::Affine3f::Identity();
        for(int i=0;i<3;i++)
            for(int j=0;j<4;j++)
                transformation(i,j)=two_T_one(i,j);
        return transformation;
    }
};

