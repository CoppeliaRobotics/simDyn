# MuJoCo plugin API reference

API functions that mainly serve to inject MuJoCo XML code

## Alphabetical list

- [simMujoco.addFlexcomp](#addFlexcomp)
- [simMujoco.addInjection](#addInjection)
- [simMujoco.composite](#composite)
- [simMujoco.getCompositeInfo](#getCompositeInfo)
- [simMujoco.getFlexcompInfo](#getFlexcompInfo)
- [simMujoco.getInfo](#getInfo)
- [simMujoco.removeInjection](#removeInjection)

## Commands

### <a name="addFlexcomp" id="addFlexcomp"></a> <a name="simMujoco.addFlexcomp" id="simMujoco.addFlexcomp"></a> simMujoco.addFlexcomp

Adds or injects a flexcomp to the MuJoCo world

#### Synopsis:

```lua
int injectionId=simMujoco.addFlexcomp(map info)
```

```python
int injectionId=simMujoco.addFlexcomp(dict info)
```

#### Parameters:

- **info** (map): an information field that contains:
  - **element**: the name of the XML element where the flexcomp XML code should be injected. Can be empty if shapeHandle is not empty
  - **shapeHandle**: the handle of the shape where the flexcomp XML code should be injected. Can be empty if element is not empty
  - **name**: the flexcomp name element. Make sure the name is unique, and possibly derived from the related shape
  - **type**: the flexcomp type element. Only grid is currently supported
  - **count**: the flexcomp count element
  - **spacing**: the flexcomp spacing element
  - **radius**: the flexcomp radius element
  - **mass**: the flexcomp mass element
  - **pose**: the flexcomp pose element
  - **pin**: the flexcomp pin element (array of indices)
  - **extraFlexcompXml**: additional xml code to be added to the flexcomp element (as attribute)
  - **extraXml**: additional xml code to be added as a flexcomp element node
  - **callback**: a callback function that is called when the MuJoCo world gets regenerated. This allows to adjust the flexcomp on-the-fly

#### Return values:

- **injectionId** (int): the id of the injection

### <a name="addInjection" id="addInjection"></a> <a name="simMujoco.addInjection" id="simMujoco.addInjection"></a> simMujoco.addInjection

Adds or injects XML code into a MuJoCo world

#### Synopsis:

```lua
int injectionId=simMujoco.addInjection(map info)
```

```python
int injectionId=simMujoco.addInjection(dict info)
```

#### Parameters:

- **info** (map): information field containing:
  - **shapeHandle**: the handle of the shape where the XML code should be injected. Can be empty if element is not empty
  - **element**: the name of the XML element where the XML code should be injected. Can be empty if shapeHandle is not empty
  - **xml**: the XML code to inject
  - **callback**: a callback function that is called when the MuJoCo world gets regenerated. This allows to adjust the xml code on-the-fly, if needed

#### Return values:

- **injectionId** (int): the id of the injection

### <a name="composite" id="composite"></a> <a name="simMujoco.composite" id="simMujoco.composite"></a> simMujoco.composite

Adds or injects a composite to the MuJoCo world

#### Synopsis:

```lua
int injectionId=simMujoco.composite(string xml,map info)
```

```python
int injectionId=simMujoco.composite(string xml,dict info)
```

#### Parameters:

- **xml** (string): the xml code corresponding to a composite object
- **info** (map): additional information about the composite:
  - **element**: the name of the XML element where the composite XML code should be injected. Can be empty if shapeHandle is not empty
  - **shapeHandle**: the handle of the shape where the composite XML code should be injected. Can be empty if element is not empty
  - **prefix**: the composite prefix string. Make sure the prefix is unique, and possibly derived from the related shape
  - **type**: the composite type. In newest Mujoco version only grid, and cable are supported types
  - **count**: the size of the composite
  - **respondableMask**: the composite respondable mask
  - **grow**: the amount the composite nodes should grow for visuals
  - **callback**: a callback function that is called when the MuJoCo world gets regenerated. This allows to adjust the composite code on-the-fly

#### Return values:

- **injectionId** (int): the id of the injection

### <a name="getCompositeInfo" id="getCompositeInfo"></a> <a name="simMujoco.getCompositeInfo" id="simMujoco.getCompositeInfo"></a> simMujoco.getCompositeInfo

Retrieves data about a composite

#### Synopsis:

```lua
map info=simMujoco.getCompositeInfo(int injectionId,int what)
```

```python
dict info=simMujoco.getCompositeInfo(int injectionId,int what)
```

#### Parameters:

- **injectionId** (int): the injection ID returned by simMujoco.composite
- **what** (int): the type of requested data:
  - **0**: the positions of the composite nodes
  - **1**: the poses of the composite nodes
  - **2**: the triangles to render the composite
  - **3**: the grown triangles to render the composite

#### Return values:

- **info** (map): the requested data

### <a name="getFlexcompInfo" id="getFlexcompInfo"></a> <a name="simMujoco.getFlexcompInfo" id="simMujoco.getFlexcompInfo"></a> simMujoco.getFlexcompInfo

Retrieves data about a flexcomp

#### Synopsis:

```lua
map info=simMujoco.getFlexcompInfo(int injectionId,int what)
```

```python
dict info=simMujoco.getCompositeInfo(int injectionId,int what)
```

#### Parameters:

- **injectionId** (int): the injection ID returned by simMujoco.addFlexcomp
- **what** (int): the type of requested data:
  - **0**: the positions of the flexcomp nodes
  - **1**: the triangles to render the flexcomp

#### Return values:

- **info** (map): the requested data

### <a name="getInfo" id="getInfo"></a> <a name="simMujoco.getInfo" id="simMujoco.getInfo"></a> simMujoco.getInfo

Retrieves general information

#### Synopsis:

```lua
string info=simMujoco.getInfo(string what)
```

```python
string info=simMujoco.getInfo(string what)
```

#### Parameters:

- **what** (string): the information type. Currently only 'nameAndIds' and 'bodies' is supported

#### Return values:

- **info** (string): the requested information

### <a name="removeInjection" id="removeInjection"></a> <a name="simMujoco.removeInjection" id="simMujoco.removeInjection"></a> simMujoco.removeInjection

Removes XML code from the MuJoCo world

#### Synopsis:

```lua
simMujoco.removeInjection(int injectionId)
```

```python
simMujoco.removeInjection(int injectionId)
```

#### Parameters:

- **injectionId** (int): the injection ID returned by simMujoco.addFlexcomp, simMujoco.composite, or simMujoco.addInjection

#### Return values:

No return values
