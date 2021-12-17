import numpy as np
import os

class sdf2urdf:

    def __init__(self):

        self.sdf_path  = 'PATH/TO/SDF/FILE'
        self.urdf_path = '/PATH/TO/URDF/FILE'

        self.sdf = self.sdf_refine(self.sdf_path)

    # Filter out meaningless strings
    def sdf_refine(self,path):

        sdf = open(path,'r')
        sdf = sdf.readlines()
        sdf = [line.strip() for line in sdf]
        sdf = np.array(sdf)

        # Remove comments to prevent adverse convertion
        nodes = self.find_nodes(sdf,'<!--','-->','generous','generous')
        comment_indices = np.zeros(1)

        for node in nodes:
            comment_index = np.arange(int(node[1]-node[0]+1)) + int(node[0])
            comment_indices = np.append(comment_indices,comment_index)
            
        comment_indices = list(comment_indices[1:].astype(int))
        sdf = np.delete(sdf,comment_indices)
        return sdf

    # Find the file path by its name
    def find(self,name,path):
        for root, dirs, files in os.walk(path):
            if name in dirs:
                return os.path.join(root, name)

    # Find any element that contains certain keyword
    def find_index(self,sdf,keyword):

        elements = [s for s in sdf if keyword in s]
        indices = np.zeros(1)

        for element in elements:
            index = np.where(sdf == element)
            indices = np.append(indices,index)

        indices = indices[1:]

        return indices

    # Find where </link name ..> ... </link> or </joint name .. > ... </joint> is
    def find_nodes(self,sdf,keyword_start,keyword_end,start_option,end_option):

        nodes = np.zeros((1,2))
        if start_option == 'finicky': # Used when finding <include> statements
            indices_start = np.where(sdf == keyword_start)[0]

        if start_option == 'generous': # Used when finding <link> & <joint> statements
            indices_start = self.find_index(sdf,keyword_start)

        if end_option == 'finicky': # Used to find link and joint statements
            indices_end   = np.where(sdf == keyword_end)[0]

        if end_option == 'generous':  # Used when removing comments
            indices_end   = self.find_index(sdf,keyword_end)

        # Group two sequential elements; first is starting index, and second is ending index
        for i in range(len(indices_start)):
            node = np.array([indices_start[i],indices_end[i]]).reshape((1,2))
            nodes = np.append(nodes,node,axis=0)

        return nodes[1:]

    # Convert </robot>
    def convert_robot(self):

        model_start = [s for s in self.sdf if 'model name=' in s][0]
        robot_start = model_start.replace("model","robot") 
        
        return np.array(robot_start + '\n').reshape((1,1))
    
    # convert </link> to </link>
    def convert_link(self,sdf):

        nodes = self.find_nodes(sdf,'link name=','</link>','generous','finicky')
        link_urdf = np.zeros(1)

        for node in nodes: # Convert in every link statement
            
            link = sdf[int(node[0]) : int(node[1])+1]

            # Catch link name
            link_name = str(link[0])
            # Catch visual name
            visual = link_name.replace('link','visual')

            # Catch mesh from mesh path
            mesh = [s for s in link if '<uri>' in s]
            if len(mesh)>0:
                mesh = str(mesh[0])
                mesh = mesh.replace("<uri>","")
                mesh = mesh.replace("</uri>","")
                mesh = mesh.replace("model","package")
                mesh = "'" + mesh + "'"

            # Catch mesh from figure description
            else:
                box      = [s for s in link if '<box>' in s]
                if len(box)>0:
                    size = str([s for s in link if '<size>' in s][0])
                    size = size.replace("<size>","")
                    size = size.replace("</size>","")

                    fig = "<box size=" + "'" + size +  "'/>\n"

                cylinder = [s for s in link if '<cylinder>' in s]
                if len(cylinder)> 0:
                    radius = str([s for s in link if '<radius>' in s][0])
                    radius = radius.replace("<radius>","")
                    radius = radius.replace("</radius>","")

                    length = str([s for s in link if '<length>' in s][0])
                    length = length.replace("<length>","")
                    length = length.replace("</length>","")

                    fig = "<cylinder radius=" + "'" + radius + "' length ='" + length +"'/>\n"

                sphere   = [s for s in link if '<sphere>' in s]
                if len(sphere) > 0:
                    radius = str([s for s in link if '<radius>' in s][0])
                    radius = radius.replace("<radius>","")
                    radius = radius.replace("</radius>","")

                    fig = "<sphere radius=" + "'" + radius +  "'/>\n"

            # Catch material
            try: 
                material = str([s for s in link if '<material>' in s][0])
                color    = str([s for s in link if '<emissive>' in s][0])
                color    = color.replace('<emissive>','')
                color    = color.replace('</emissive>','')
                color    = "<material name='noname'>" + '\n'+ "    <color rgba='" + color + "'/>"+"\n" + '</material>' +'\n'

            except:
                color = ''
                pass

            # Catch scale
            try: 
                scale = str([s for s in link if '<scale>' in s][0])
                scale = scale.replace("<scale>","")
                scale = scale.replace("</scale>","")
                scale = " scale='" + scale  + "'"
            except:
                scale = ''
                pass

            # Catch pose > origin
            pose = [s for s in link if '<pose>' in s]
            if len(pose)>0:
                pose = str(pose[0])
                pose = pose.replace("<pose>","")
                pose = pose.replace("</pose>","")
                x,y,z,= pose.split()[0:3]
                roll,pitch,yaw = pose.split()[3:6]
                origin = "'" + x + " " + y + " " + z + "' rpy='" + roll + " " + pitch + " " + yaw + " " + "'"
            else:
                origin = ' '
            
            # Rewrite in urdf form 
            link1 = np.array([
            link_name + '\n',
            ' ' + visual + "\n",
            '  <origin ' + origin + '/>\n',
            '  <geometry>\n'])
            if len(mesh)>0:
                link2 = np.array(['   <mesh filename =' + mesh + scale + '/>\n',])
            else:
                link2 = np.array([fig])
            link3 = np.array(['  <geometry>\n'])
            link4 = np.array([color])
            link5 = np.array([
            ' </visual>\n',
            '</link>\n'])
            link_temp = np.concatenate((link1,link2,link3,link4,link5),axis=0)
            link_urdf = np.append(link_urdf,link_temp)
        link_urdf = link_urdf[1:]
        
        return link_urdf

    # convert </include> to </link>
    def convert_include(self):

        nodes = self.find_nodes(self.sdf,'<include>','</include>','finicky','finicky')
        include_urdf = np.zeros(1)

        for node in nodes: # Convert in every include statement
            
            include = self.sdf[int(node[0]) : int(node[1])+1]
            # Catch name
            name = str([s for s in include if '<name>' in s][0])
            name = name.replace("<name>","")
            name = name.replace("</name>","")

            # Define link name 
            link_name   = '<link name=' + "'" + name + "'" + '>'

            # Define visual name
            visual = '<visual name=' + "'" + name + "'" + '>'

            # Catch origin
            pose = str([s for s in include if '<pose>' in s][0])
            pose = pose.replace("<pose>","")
            pose = pose.replace("</pose>","")
            x,y,z,= pose.split()[0:3]
            roll,pitch,yaw = pose.split()[3:6]
            origin = "'" + x + " " + y + " " + z + "' rpy='" + roll + " " + pitch + " " + yaw + " " + "'"
            # Get sdf path
            model_name = str([s for s in include if '<uri>' in s][0])
            model_name = model_name.replace("<uri>model://","")
            model_name = model_name.replace("</uri>","")

            # Open inner sdf file 
            model_path = self.find(model_name,'.')
            sdf_path = model_path + '/model.sdf'

            # Rewrite in urdf form
            sdf = self.sdf_refine(sdf_path)
            include_temp = self.convert_link(sdf)

            include_temp[0] = link_name + '\n'
            include_temp[1] = ' ' + visual + "\n"
            include_temp[2] = '  <origin ' + origin + '/>\n'

            include_urdf = np.append(include_urdf,include_temp)

        include_urdf = include_urdf[1:]
        
        return include_urdf

    # convert </joint> 
    def convert_joint(self):
        
        nodes = self.find_nodes(self.sdf,'joint name=','</joint>','generous','finicky')        
        joint_urdf = np.zeros(1)

        for node in nodes: # Convert in every joint statement
            
            statement = self.sdf[int(node[0]) : int(node[1])+1]
            # Catch joint name
            joint = statement[0]
            
            # Catch axis
            try: 
                xyz = str([s for s in statement if '<xyz>' in s][0])
                xyz = xyz.replace("<xyz>","")
                xyz = xyz.replace("</xyz>","")
                xyz = ' <axis xyz=' + "'" + xyz + "'/>\n"

            except:
                xyz = ''
            
            # Catch parent link
            parent = str([s for s in statement if '<parent>' in s][0])
            parent = parent.replace("<parent>","")
            parent = parent.replace("</parent>","")
            parent = parent.replace('::link','')

            # Catch child link
            child = str([s for s in statement if '<child>' in s][0])
            child = child.replace("<child>","")
            child = child.replace("</child>","")
            child = child.replace("::link",'')

            # Rewrite in urdf form 
            joint_temp = np.array([
            joint + '\n',
            xyz,
            ' <parent link=' + "'" + parent + "'" + '/>\n',
            ' <child link='  + "'" + child  + "'" + '/>\n',
            '</joint>\n'])

            joint_urdf = np.append(joint_urdf,joint_temp)
        
        joint_urdf = joint_urdf[1:]
       
        return joint_urdf

    def write_urdf(self):

        lines = np.zeros(1).reshape((1,1))

        urdf = open(self.urdf_path,'w')
        # Reset the file
        urdf.truncate(0) 
        # Define xml version
        lines = np.append(lines,np.array('<?xml version="1.0"?>' + '\n').reshape((1,1)),axis=1)
        # </robot> start
        lines = np.append(lines,self.convert_robot().reshape((1,1)),axis=1)
        # </link>
        for i in range(len(self.convert_link(self.sdf))):
            lines = np.append(lines,np.array(self.convert_link(self.sdf)[i]).reshape((1,1)),axis=1)
        # </include>
        for i in range(len(self.convert_include())):
            lines = np.append(lines,np.array(self.convert_include()[i]).reshape((1,1)),axis=1)
        # </joint>
        for i in range(len(self.convert_joint())):
            lines = np.append(lines,np.array(self.convert_joint()[i]).reshape((1,1)),axis=1)
        # </robot> end
        lines = np.append(lines,np.array('</robot>').reshape((1,1)),axis=1)

        
        lines = list(lines[0][1:])
        

        urdf.writelines(lines)
        return urdf

if __name__=='__main__':
    a = sdf2urdf()
    a.write_urdf()
